#include <AP_gtest.h>

#include <AP_ContactSensor/AP_ContactDetector.h>

TEST(ContactDetector, RequiresConsecutiveSamples)
{
    AP_ContactDetector detector;
    detector.configure(0.5f, 0.25f, 3);

    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.6f));
    EXPECT_TRUE(detector.contact_candidate());
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.4f));
    EXPECT_FALSE(detector.contact_candidate());
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.6f));
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.7f));
    EXPECT_EQ(AP_ContactDetector::Event::CONTACT, detector.update(0.8f));
    EXPECT_TRUE(detector.contact());
}

TEST(ContactDetector, ReleaseUsesHysteresis)
{
    AP_ContactDetector detector;
    detector.configure(0.5f, 0.25f, 3);
    detector.reset(true);

    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.3f));
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.2f));
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.3f));
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.2f));
    EXPECT_EQ(AP_ContactDetector::Event::NONE, detector.update(0.2f));
    EXPECT_EQ(AP_ContactDetector::Event::RELEASE, detector.update(0.2f));
    EXPECT_FALSE(detector.contact());
}

AP_GTEST_MAIN()
