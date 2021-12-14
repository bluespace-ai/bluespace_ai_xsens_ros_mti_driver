#include "gtest/gtest.h"

#include "xdautils.h"

TEST(Xda, test_get_xs_data_identifier_by_name)
{
    XsDataIdentifier id;
    auto result = get_xs_data_identifier_by_name("XDI_Acceleration", id);
    ASSERT_TRUE(result);
    ASSERT_EQ(id, XDI_Acceleration);
}

TEST(Xda, test_get_xs_data_identifier_by_name_non_existing)
{
    XsDataIdentifier id;
    auto result = get_xs_data_identifier_by_name("XDI_Acceleration2", id);
    ASSERT_FALSE(result);
}

TEST(Xda, test_get_xs_data_identifier_name)
{
    auto name = get_xs_data_identifier_name(XDI_Acceleration);
    ASSERT_EQ(name, "XDI_Acceleration");
}

TEST(Xda, test_parse_line)
{
    std::string name;
    int value;
    auto result = parseConfigLine("XDI_Acceleration=1337", name, value);
    ASSERT_TRUE(result);
    ASSERT_EQ(name, "XDI_Acceleration");
    ASSERT_EQ(value, 1337);
}

TEST(Xda, test_parse_line_error)
{
    std::string name;
    int value;
    auto result = parseConfigLine("XDI_Acceleration", name, value);
    ASSERT_FALSE(result);
}

TEST(Xda, test_parse_line_error2)
{
    std::string name;
    int value;
    auto result = parseConfigLine("XDI_Acceleration=13=12", name, value);
    ASSERT_FALSE(result);
}
