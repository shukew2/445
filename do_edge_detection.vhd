-- read buffer 2 content, which stores taken snapshot
-- apply Sobel filter for edge detection
-- modified version:
-- 1. add ROI-based foreground detection
-- 2. add ROI-based scratch / damage detection
-- 3. output frame-level classification signals

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_unsigned.all;

entity do_edge_detection is
Port (
    rst_i : in STD_LOGIC;
    clk_i : in STD_LOGIC;

    enable_sobel_filter : in STD_LOGIC;
    led_sobel_done : out STD_LOGIC;

    rdaddr_buf1 : OUT STD_LOGIC_VECTOR (16 downto 0);
    din_buf1 : IN std_logic_vector(11 downto 0);

    wraddr_buf2 : OUT STD_LOGIC_VECTOR (16 downto 0);
    dout_buf2 : OUT std_logic_vector(11 downto 0);
    we_buf2 : OUT std_logic;

    frame_package_present_o : out STD_LOGIC;
    frame_scratch_o         : out STD_LOGIC
);
end do_edge_detection;

architecture my_behavioral of do_edge_detection is

    COMPONENT edge_sobel_wrapper
    generic (
        DATA_WIDTH : integer := 8
    );
    Port (
        clk : in STD_LOGIC;
        fsync_in : in STD_LOGIC;
        rsync_in : in STD_LOGIC;
        pdata_in : in STD_LOGIC_VECTOR (DATA_WIDTH-1 downto 0);
        fsync_out : out STD_LOGIC;
        rsync_out : out STD_LOGIC;
        pdata_out : out STD_LOGIC_VECTOR (DATA_WIDTH-1 downto 0)
    );
    end COMPONENT;

    constant START_SOBEL_FILTER_ST : std_logic_vector(2 downto 0) := "000";
    constant GET_PIXEL_DATA_ST     : std_logic_vector(2 downto 0) := "001";
    constant STALL_1_CYCLE_ST      : std_logic_vector(2 downto 0) := "010";
    constant STALL_2_CYCLE_ST      : std_logic_vector(2 downto 0) := "011";
    constant SEND_PIXEL_DATA_ST    : std_logic_vector(2 downto 0) := "100";
    constant DONE_ST               : std_logic_vector(2 downto 0) := "101";
    constant IDLE_ST               : std_logic_vector(2 downto 0) := "110";

    constant NUM_PIXELS : std_logic_vector(16 downto 0) := std_logic_vector(to_unsigned(76799, 17));

    signal led_done_r: std_logic := '0';

    signal rdaddr_buf1_r : STD_LOGIC_VECTOR(16 downto 0) := (others => '0');
    signal din_buf1_r : std_logic_vector(7 downto 0);

    signal wraddr_buf2_r : STD_LOGIC_VECTOR(16 downto 0) := (others => '0');
    signal dout_buf2_r : std_logic_vector(7 downto 0) := (others => '0');
    signal we_buf2_r : std_logic := '0';

    signal rd_cntr: std_logic_vector(16 downto 0) := (others => '0');
    signal wr_cntr: std_logic_vector(16 downto 0) := (others => '0');

    signal state: std_logic_vector(2 downto 0) := IDLE_ST;

    signal hsync_dummy: std_logic := '0';
    signal vsync_dummy: std_logic := '0';
    signal hsync_delayed: std_logic;
    signal vsync_delayed: std_logic;

    signal ColsCounter : std_logic_vector(8 downto 0) := "000000000";
    signal clk_div2: std_logic := '0';

    --------------------------------------------------------------------
    -- New signals for package detection and scratch classification
    --------------------------------------------------------------------

    -- ROI position for 320 x 240 image
    -- You can adjust this region according to the real camera view.
    constant ROI_X_MIN : integer := 80;
    constant ROI_X_MAX : integer := 239;
    constant ROI_Y_MIN : integer := 60;
    constant ROI_Y_MAX : integer := 179;
	 

    -- Pixel thresholds
    constant FG_THRESHOLD   : unsigned(7 downto 0) := to_unsigned(80, 8);
	 constant EDGE_THRESHOLD : unsigned(7 downto 0) := to_unsigned(200, 8);

    -- Count thresholds
    -- These values should be tuned using real camera images.
    constant PACKAGE_PRESENT_THRESHOLD : unsigned(15 downto 0) := to_unsigned(4000, 16);
	 constant SCRATCH_THRESHOLD         : unsigned(15 downto 0) := to_unsigned(900, 16);

    signal roi_x : integer range 0 to 319 := 0;
    signal roi_y : integer range 0 to 239 := 0;

    signal foreground_count : unsigned(15 downto 0) := (others => '0');
    signal scratch_count    : unsigned(15 downto 0) := (others => '0');

    signal frame_package_present_r : STD_LOGIC := '0';
    signal frame_scratch_r         : STD_LOGIC := '0';

    signal in_roi : STD_LOGIC := '0';

begin

    led_sobel_done <= led_done_r;

    rdaddr_buf1 <= rdaddr_buf1_r;

    din_buf1_r <= (din_buf1(3 downto 0) & "0000");

    we_buf2 <= we_buf2_r;
    wraddr_buf2 <= wraddr_buf2_r;

    dout_buf2 <= (dout_buf2_r(7 downto 4) &
                  dout_buf2_r(7 downto 4) &
                  dout_buf2_r(7 downto 4));

    frame_package_present_o <= frame_package_present_r;
    frame_scratch_o <= frame_scratch_r;

    in_roi <= '1' when
        roi_x >= ROI_X_MIN and roi_x <= ROI_X_MAX and
        roi_y >= ROI_Y_MIN and roi_y <= ROI_Y_MAX
    else '0';

    Inst_edge_sobel_wrapper: edge_sobel_wrapper
    PORT MAP (
        clk => clk_div2,
        fsync_in => vsync_dummy,
        rsync_in => hsync_dummy,
        pdata_in => din_buf1_r,
        fsync_out => vsync_delayed,
        rsync_out => hsync_delayed,
        pdata_out => dout_buf2_r
    );

    --------------------------------------------------------------------
    -- Clock divider for Sobel wrapper
    --------------------------------------------------------------------
    divide_clk_by_2_proc : process(clk_i, rst_i)
    begin
        if rst_i = '1' then
            clk_div2 <= '0';
        elsif rising_edge(clk_i) then
            clk_div2 <= not clk_div2;
        end if;
    end process;

    --------------------------------------------------------------------
    -- Main Sobel and frame-statistics FSM
    --------------------------------------------------------------------
    process (clk_i)
    begin
        if rising_edge (clk_i) then

            if (rst_i = '1') then

                state <= IDLE_ST;
                led_done_r <= '0';

                rd_cntr <= (others => '0');
                wr_cntr <= (others => '0');

                we_buf2_r <= '0';

                rdaddr_buf1_r <= (others => '0');
                wraddr_buf2_r <= (others => '0');

                vsync_dummy <= '0';
                hsync_dummy <= '0';

                ColsCounter <= "000000000";

                roi_x <= 0;
                roi_y <= 0;

                foreground_count <= (others => '0');
                scratch_count <= (others => '0');

                frame_package_present_r <= '0';
                frame_scratch_r <= '0';

            elsif (enable_sobel_filter = '1' and state = IDLE_ST) then

                state <= START_SOBEL_FILTER_ST;
                led_done_r <= '0';

                rd_cntr <= (others => '0');
                wr_cntr <= (others => '0');

                we_buf2_r <= '1';

                rdaddr_buf1_r <= (others => '0');
                wraddr_buf2_r <= (others => '0');

                vsync_dummy <= '1';
                hsync_dummy <= '0';

                ColsCounter <= "000000000";

                roi_x <= 0;
                roi_y <= 0;

                foreground_count <= (others => '0');
                scratch_count <= (others => '0');

                frame_package_present_r <= '0';
                frame_scratch_r <= '0';

            else

                case state is

                    when START_SOBEL_FILTER_ST =>

                        state <= GET_PIXEL_DATA_ST;
                        led_done_r <= '0';

                        rd_cntr <= (others => '0');
                        wr_cntr <= (others => '0');

                        we_buf2_r <= '1';

                        rdaddr_buf1_r <= (others => '0');
                        wraddr_buf2_r <= (others => '0');

                        vsync_dummy <= '1';
                        hsync_dummy <= '0';

                        ColsCounter <= "000000000";

                        roi_x <= 0;
                        roi_y <= 0;

                        foreground_count <= (others => '0');
                        scratch_count <= (others => '0');

                        frame_package_present_r <= '0';
                        frame_scratch_r <= '0';

                    when GET_PIXEL_DATA_ST =>

                        state <= SEND_PIXEL_DATA_ST;

                        rdaddr_buf1_r <= rdaddr_buf1_r + 1;

                        if (rd_cntr > 323) then
                            wraddr_buf2_r <= wraddr_buf2_r + 1;
                            wr_cntr <= wr_cntr + 1;
                        else
                            wraddr_buf2_r <= (others => '0');
                            wr_cntr <= (others => '0');
                        end if;

                    when SEND_PIXEL_DATA_ST =>

                        if (wr_cntr < NUM_PIXELS) then

                            rd_cntr <= rd_cntr + 1;

                            ------------------------------------------------
                            -- ROI statistics
                            ------------------------------------------------
                            if in_roi = '1' then

                                -- Detect package body against black background
                                if unsigned(din_buf1_r) > FG_THRESHOLD then
                                    if foreground_count < to_unsigned(65535, 16) then
                                        foreground_count <= foreground_count + 1;
                                    end if;
                                end if;

                                -- Detect strong edge pixels as scratch candidates
                                if unsigned(dout_buf2_r) > EDGE_THRESHOLD then
                                    if scratch_count < to_unsigned(65535, 16) then
                                        scratch_count <= scratch_count + 1;
                                    end if;
                                end if;

                            end if;

                            ------------------------------------------------
                            -- Update ROI coordinate
                            ------------------------------------------------
                            if roi_x < 319 then
                                roi_x <= roi_x + 1;
                            else
                                roi_x <= 0;

                                if roi_y < 239 then
                                    roi_y <= roi_y + 1;
                                else
                                    roi_y <= 0;
                                end if;
                            end if;

                            ------------------------------------------------
                            -- Original row sync logic
                            ------------------------------------------------
                            if ColsCounter < 319 then
                                ColsCounter <= ColsCounter + 1;
                                hsync_dummy <= '1';
                                state <= GET_PIXEL_DATA_ST;
                            else
                                ColsCounter <= "000000000";
                                hsync_dummy <= '0';
                                state <= STALL_1_CYCLE_ST;
                            end if;

                        else
                            state <= DONE_ST;
                        end if;

                    when STALL_1_CYCLE_ST =>

                        state <= STALL_2_CYCLE_ST;

                    when STALL_2_CYCLE_ST =>

                        state <= GET_PIXEL_DATA_ST;
                        hsync_dummy <= '1';

                    when DONE_ST =>

                        state <= DONE_ST;
                        led_done_r <= '1';
                        we_buf2_r <= '0';

                        if foreground_count > PACKAGE_PRESENT_THRESHOLD then
                            frame_package_present_r <= '1';
                        else
                            frame_package_present_r <= '0';
                        end if;

                        if scratch_count > SCRATCH_THRESHOLD then
                            frame_scratch_r <= '1';
                        else
                            frame_scratch_r <= '0';
                        end if;

                    when others =>

                        state <= IDLE_ST;
                        led_done_r <= '0';
                        we_buf2_r <= '0';

                end case;

            end if;

        end if;
    end process;

end my_behavioral;