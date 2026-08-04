#include <AP_gtest.h>

#include <AP_ContactSensor/AP_ADM002.h>
#include <string.h>

#if AP_CONTACT_SENSOR_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class AP_ADM002Test
{
public:
    static void make_frame(uint8_t *frame, uint8_t status, uint32_t magnitude_g)
    {
        frame[0] = status;
        frame[1] = uint8_t(magnitude_g >> 16);
        frame[2] = uint8_t(magnitude_g >> 8);
        frame[3] = uint8_t(magnitude_g);
        frame[4] = AP_ADM002::checksum(frame, 4);
    }

    static bool feed(AP_ADM002 &sensor, const uint8_t *data, uint8_t len)
    {
        if (uint16_t(sensor._rx_len) + len > AP_ADM002::RX_BUF_LEN) {
            return false;
        }
        memcpy(&sensor._rx_buf[sensor._rx_len], data, len);
        sensor._rx_len += len;
        return sensor.parse_buffer();
    }

    static int32_t weight_g(const AP_ADM002 &sensor)
    {
        return sensor._weight_g;
    }
    static uint32_t sequence(const AP_ADM002 &sensor)
    {
        return sensor._sample_sequence;
    }
    static uint8_t checksum(const uint8_t *data, uint8_t len)
    {
        return AP_ADM002::checksum(data, len);
    }
    static void set_timestamp(AP_ADM002 &sensor, uint32_t timestamp_ms)
    {
        sensor._last_update_ms = timestamp_ms;
    }
    static void expect_config_ack(AP_ADM002 &sensor, AP_ADM002::ConfigCommand command, uint8_t function)
    {
        sensor._config_command = command;
        sensor._config_state = AP_ADM002::ConfigState::PENDING;
        sensor._config_ack_function = function;
    }
};

TEST(ADM002, ParsesPositiveAndNegativeFrames)
{
    AP_ADM002 sensor;
    uint8_t frame[5];

    AP_ADM002Test::make_frame(frame, 0x03, 20000);
    EXPECT_TRUE(AP_ADM002Test::feed(sensor, frame, sizeof(frame)));
    EXPECT_EQ(20000, AP_ADM002Test::weight_g(sensor));

    AP_ADM002Test::make_frame(frame, 0x02, 1234);
    EXPECT_TRUE(AP_ADM002Test::feed(sensor, frame, sizeof(frame)));
    EXPECT_EQ(-1234, AP_ADM002Test::weight_g(sensor));
    EXPECT_EQ(2U, AP_ADM002Test::sequence(sensor));
}

TEST(ADM002, RejectsChecksumAndReservedStatusBits)
{
    uint8_t frame[5];

    AP_ADM002 sensor;
    AP_ADM002Test::make_frame(frame, 0x01, 100);
    frame[4]++;
    EXPECT_FALSE(AP_ADM002Test::feed(sensor, frame, sizeof(frame)));
    EXPECT_EQ(0U, AP_ADM002Test::sequence(sensor));

    AP_ADM002 reserved_status_sensor;
    AP_ADM002Test::make_frame(frame, 0x05, 100);
    EXPECT_FALSE(AP_ADM002Test::feed(reserved_status_sensor, frame, sizeof(frame)));
    EXPECT_EQ(0U, AP_ADM002Test::sequence(reserved_status_sensor));
}

TEST(ADM002, ResynchronisesAndKeepsNewestSample)
{
    AP_ADM002 sensor;
    uint8_t stream[12] {0xFF, 0xAA};
    AP_ADM002Test::make_frame(&stream[2], 0x01, 100);
    AP_ADM002Test::make_frame(&stream[7], 0x01, 200);

    EXPECT_TRUE(AP_ADM002Test::feed(sensor, stream, sizeof(stream)));
    EXPECT_EQ(200, AP_ADM002Test::weight_g(sensor));
    EXPECT_EQ(2U, AP_ADM002Test::sequence(sensor));
    EXPECT_GE(sensor.checksum_error_count(), 2U);
}

TEST(ADM002, UsesDocumentedEnableCommandChecksum)
{
    const uint8_t command[] {0x01, 0x28, 0x01, 0x01};
    EXPECT_EQ(0x2B, AP_ADM002Test::checksum(command, sizeof(command)));
}

TEST(ADM002, ConvertsGramsToNewtonsAndAppliesSoftwareTare)
{
    AP_ADM002 sensor;
    uint8_t frame[5];
    AP_ADM002Test::make_frame(frame, 0x01, 1000);
    ASSERT_TRUE(AP_ADM002Test::feed(sensor, frame, sizeof(frame)));
    AP_ADM002Test::set_timestamp(sensor, 1);

    AP_ContactSensor::ForceSample sample;
    ASSERT_TRUE(sensor.get_force_sample(sample));
    EXPECT_NEAR(GRAVITY_MSS, sample.tool_force_n, 1.0e-5f);
    ASSERT_TRUE(sensor.tare());
    ASSERT_TRUE(sensor.get_force_sample(sample));
    EXPECT_FLOAT_EQ(0.0f, sample.tool_force_n);
}

TEST(ADM002, ParsesConfigAckBetweenStreamFrames)
{
    AP_ADM002 sensor;
    uint8_t stream[13];
    AP_ADM002Test::make_frame(&stream[0], 0x01, 100);
    stream[5] = 0x01;
    stream[6] = 0x17;
    stream[7] = 0x18;
    AP_ADM002Test::make_frame(&stream[8], 0x01, 200);
    AP_ADM002Test::expect_config_ack(sensor, AP_ADM002::ConfigCommand::FULL_SCALE, 0x17);

    EXPECT_TRUE(AP_ADM002Test::feed(sensor, stream, sizeof(stream)));
    EXPECT_EQ(AP_ADM002::ConfigState::SUCCESS, sensor.config_state());
    EXPECT_EQ(1U, sensor.config_sequence());
    EXPECT_EQ(200, AP_ADM002Test::weight_g(sensor));
    EXPECT_EQ(2U, AP_ADM002Test::sequence(sensor));
}

#endif

AP_GTEST_MAIN()
