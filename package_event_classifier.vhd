library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity package_event_classifier is
    Port (
        clk_i                  : in  STD_LOGIC;
        rst_i                  : in  STD_LOGIC;

        frame_done_i            : in  STD_LOGIC;
        frame_package_present_i : in  STD_LOGIC;
        frame_scratch_i         : in  STD_LOGIC;

        package_in_region_o     : out STD_LOGIC;
        result_valid_o          : out STD_LOGIC;
        damaged_o               : out STD_LOGIC
    );
end package_event_classifier;

architecture Behavioral of package_event_classifier is

    type state_type is (IDLE, IN_PACKAGE);
    signal state : state_type := IDLE;

    signal present_cnt : unsigned(2 downto 0) := (others => '0');
    signal absent_cnt  : unsigned(2 downto 0) := (others => '0');

    signal frame_done_d     : STD_LOGIC := '0';
    signal frame_done_pulse : STD_LOGIC := '0';

    signal valid_hold_cnt : unsigned(23 downto 0) := (others => '0');

    -- Count how many valid package frames have been processed
    signal package_frame_cnt : unsigned(7 downto 0) := (others => '0');

    -- Count how many frames are detected as scratch during one package pass
    signal scratch_frame_cnt : unsigned(7 downto 0) := (others => '0');

    constant PRESENT_CONFIRM_FRAMES : unsigned(2 downto 0) := "010"; -- 2 frames
    constant ABSENT_CONFIRM_FRAMES  : unsigned(2 downto 0) := "010"; -- 2 frames

    -- Ignore first few frames after package enters ROI.
    -- This avoids detecting package boundary as scratch.
    constant IGNORE_START_FRAMES : unsigned(7 downto 0) := to_unsigned(2, 8);

    -- Final damage decision threshold.
    -- Larger value = harder to judge as damaged.
    constant DAMAGE_FRAME_THRESHOLD : unsigned(7 downto 0) := to_unsigned(4, 8);

    -- 25 MHz clock, about 0.2 s LED hold time
    constant VALID_HOLD_TIME : unsigned(23 downto 0) := to_unsigned(5000000, 24);

begin

    frame_done_pulse <= frame_done_i and (not frame_done_d);

    process(clk_i)
    begin
        if rising_edge(clk_i) then

            if rst_i = '1' then

                state <= IDLE;

                present_cnt <= (others => '0');
                absent_cnt <= (others => '0');

                frame_done_d <= '0';

                valid_hold_cnt <= (others => '0');

                package_frame_cnt <= (others => '0');
                scratch_frame_cnt <= (others => '0');

                package_in_region_o <= '0';
                result_valid_o <= '0';
                damaged_o <= '0';

            else

                frame_done_d <= frame_done_i;

                --------------------------------------------------
                -- Hold result_valid long enough for LED debugging
                --------------------------------------------------
                if valid_hold_cnt > 0 then
                    valid_hold_cnt <= valid_hold_cnt - 1;
                    result_valid_o <= '1';
                else
                    result_valid_o <= '0';
                end if;

                --------------------------------------------------
                -- Process only once for each completed frame
                --------------------------------------------------
                if frame_done_pulse = '1' then

                    case state is

                        --------------------------------------------------
                        -- No package detected yet
                        --------------------------------------------------
                        when IDLE =>

                            package_in_region_o <= '0';
                            absent_cnt <= (others => '0');

                            package_frame_cnt <= (others => '0');
                            scratch_frame_cnt <= (others => '0');

                            if frame_package_present_i = '1' then

                                if present_cnt < PRESENT_CONFIRM_FRAMES then
                                    present_cnt <= present_cnt + 1;
                                end if;

                                if present_cnt >= PRESENT_CONFIRM_FRAMES - 1 then

                                    state <= IN_PACKAGE;
                                    package_in_region_o <= '1';

                                    -- Clear old result when a new package starts
                                    damaged_o <= '0';

                                    package_frame_cnt <= (others => '0');
                                    scratch_frame_cnt <= (others => '0');

                                end if;

                            else
                                present_cnt <= (others => '0');
                            end if;

                        --------------------------------------------------
                        -- Package is inside ROI
                        --------------------------------------------------
                        when IN_PACKAGE =>

                            package_in_region_o <= '1';

                            --------------------------------------------------
                            -- Count package frames
                            --------------------------------------------------
                            if package_frame_cnt < to_unsigned(255, 8) then
                                package_frame_cnt <= package_frame_cnt + 1;
                            end if;

                            --------------------------------------------------
                            -- Count scratch frames only after initial frames
                            --------------------------------------------------
                            if package_frame_cnt > IGNORE_START_FRAMES then

                                if frame_scratch_i = '1' then
                                    if scratch_frame_cnt < to_unsigned(255, 8) then
                                        scratch_frame_cnt <= scratch_frame_cnt + 1;
                                    end if;
                                end if;

                            end if;

                            --------------------------------------------------
                            -- Check whether package has left ROI
                            --------------------------------------------------
                            if frame_package_present_i = '0' then

                                if absent_cnt < ABSENT_CONFIRM_FRAMES then
                                    absent_cnt <= absent_cnt + 1;
                                end if;

                                if absent_cnt >= ABSENT_CONFIRM_FRAMES - 1 then

                                    --------------------------------------------------
                                    -- Final binary classification
                                    --------------------------------------------------
                                    if scratch_frame_cnt >= DAMAGE_FRAME_THRESHOLD then
                                        damaged_o <= '1';
                                    else
                                        damaged_o <= '0';
                                    end if;

                                    valid_hold_cnt <= VALID_HOLD_TIME;
                                    result_valid_o <= '1';

                                    state <= IDLE;
                                    package_in_region_o <= '0';

                                    present_cnt <= (others => '0');
                                    absent_cnt <= (others => '0');

                                    package_frame_cnt <= (others => '0');
                                    scratch_frame_cnt <= (others => '0');

                                end if;

                            else
                                absent_cnt <= (others => '0');
                            end if;

                    end case;

                end if;

            end if;

        end if;
    end process;

end Behavioral;