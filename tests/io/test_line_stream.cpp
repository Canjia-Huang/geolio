//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/22.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <geolio/io/line_stream.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief Writes the given lines to a temporary file and returns its path.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] lines the lines of the file
         */
        std::string write_lines_file(const std::string& test_name, const std::vector<std::string>& lines) {
            const std::string filepath = (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".txt")
            ).string();

            std::ofstream out(filepath);
            for(const auto& line : lines) {
                out << line << "\n";
            }
            return filepath;
        }

        /**
         * @brief Converts a single field with field_as_appro_double().
         * @param[in] field the field to convert
         * @return the converted value
         * @exception std::logic_error if the field cannot be converted
         */
        double appro_double_of(const std::string& field) {
            const std::string filepath = write_lines_file("appro_double_of", {field});
            double result = 0.0;
            {
                geolio::LineInput in(filepath);
                EXPECT_TRUE(in.OK());
                EXPECT_TRUE(in.get_line());
                in.get_fields();
                EXPECT_EQ(in.nb_fields(), 1);
                result = in.field_as_appro_double(0);
            }
            std::filesystem::remove(filepath);
            return result;
        }

        /**
         * @brief Converts the rational num / den with field_as_appro_double().
         */
        double appro_double_of(const std::string& num, const std::string& den) {
            return appro_double_of(num + "/" + den);
        }

        /**
         * @brief Converts a single field with field_as_double().
         * @param[in] field the field to convert
         * @return the converted value
         * @exception std::logic_error if the field cannot be converted
         */
        double double_of(const std::string& field) {
            const std::string filepath = write_lines_file("double_of", {field});
            double result = 0.0;
            {
                geolio::LineInput in(filepath);
                EXPECT_TRUE(in.OK());
                EXPECT_TRUE(in.get_line());
                in.get_fields();
                EXPECT_EQ(in.nb_fields(), 1);
                result = in.field_as_double(0);
            }
            std::filesystem::remove(filepath);
            return result;
        }

        /**
         * @brief Returns true if field_as_double() rejects the given field.
         */
        bool double_rejects(const std::string& field) {
            const std::string filepath = write_lines_file("double_rejects", {field});
            bool thrown = false;
            {
                geolio::LineInput in(filepath);
                EXPECT_TRUE(in.OK());
                EXPECT_TRUE(in.get_line());
                in.get_fields();
                EXPECT_EQ(in.nb_fields(), 1);
                try {
                    static_cast<void>(in.field_as_double(0));
                } catch(const std::logic_error&) {
                    thrown = true;
                }
            }
            std::filesystem::remove(filepath);
            return thrown;
        }
    }

    /**
     * @brief Plain decimal literals and small rationals are read as expected.
     */
    TEST(LineStream, ApproximateDoubleReadsPlainAndFractionalFields) {
        EXPECT_DOUBLE_EQ(appro_double_of("0"), 0.0);
        EXPECT_DOUBLE_EQ(appro_double_of("42"), 42.0);
        EXPECT_DOUBLE_EQ(appro_double_of("-1.5"), -1.5);
        EXPECT_DOUBLE_EQ(appro_double_of(".5"), 0.5);
        EXPECT_DOUBLE_EQ(appro_double_of("5."), 5.0);
        EXPECT_DOUBLE_EQ(appro_double_of("1e10"), 1e10);
        EXPECT_DOUBLE_EQ(appro_double_of("-2.5E-3"), -2.5e-3);
        EXPECT_DOUBLE_EQ(appro_double_of("1/3"), 1.0 / 3.0);
        EXPECT_DOUBLE_EQ(appro_double_of("-1/3"), -1.0 / 3.0);
        EXPECT_DOUBLE_EQ(appro_double_of("1/-3"), -1.0 / 3.0);
        EXPECT_DOUBLE_EQ(appro_double_of("0/5"), 0.0);
        EXPECT_DOUBLE_EQ(appro_double_of("1e-400"), 0.0);
    }

    /**
     * @brief A rational whose parts are too large for a long double is reduced
     *        to its quotient instead of being rejected.
     * @details The previous implementation converted the numerator and the
     *          denominator separately with strtold(), so a part outside the
     *          range of long double raised ERANGE and the whole field was
     *          rejected, even though the quotient is an ordinary number.
     *          Hexahedral integer grid mappings write such rationals (the
     *          denominator of a corner coordinate easily has 300+ digits).
     */
    TEST(LineStream, ApproximateDoubleReadsRationalsBeyondLongDoubleRange) {
        // 7.25e399 / 1e399 == 7.25, both parts overflow long double.
        EXPECT_NEAR(
            appro_double_of("725" + std::string(397, '0'), "1" + std::string(399, '0')),
            7.25, 1e-12);

        // The same with a sign and with more digits than a double can hold.
        EXPECT_NEAR(
            appro_double_of("-725" + std::string(500, '0'), "1" + std::string(502, '0')),
            -7.25, 1e-12);

        // 1e299 / 1e499 == 1e-200: a tiny quotient of two huge parts.
        EXPECT_NEAR(
            appro_double_of("1" + std::string(299, '0'), "1" + std::string(499, '0')),
            1e-200, 1e-210);

        // A plain literal with more digits than a double significand holds,
        // whose value still fits in a double: 0.<300 zeros>725 == 7.25e-301.
        EXPECT_NEAR(
            appro_double_of("0." + std::string(300, '0') + "725"),
            7.25e-301, 1e-310);
    }

    /**
     * @brief Fields that are not numbers, and numbers that do not fit in a
     *        double, throw instead of silently producing garbage.
     */
    TEST(LineStream, ApproximateDoubleRejectsInvalidFields) {
        std::vector<std::string> invalid_fields = {
            "abc",     // Not a number.
            "1/",      // Missing denominator.
            "/2",      // Missing numerator.
            "1//2",    // Two '/' characters.
            "1/0",     // Division by zero.
            "0/0",     // Division by zero.
            "1e",      // Missing exponent.
            "1.2.3",   // Two decimal points.
            "1e400",   // Overflows double.
            std::string(400, '9'), // Overflows double.
        };

        for(const auto& field : invalid_fields) {
            SCOPED_TRACE(field.substr(0, 16));
            bool thrown = false;
            const std::string filepath = write_lines_file("ApproximateDoubleRejectsInvalidFields", {field});
            {
                geolio::LineInput in(filepath);
                EXPECT_TRUE(in.OK());
                EXPECT_TRUE(in.get_line());
                in.get_fields();
                EXPECT_EQ(in.nb_fields(), 1);
                try {
                    static_cast<void>(in.field_as_appro_double(0));
                } catch(const std::logic_error&) {
                    thrown = true;
                }
            }
            std::filesystem::remove(filepath);
            EXPECT_TRUE(thrown);
        }
    }

    /**
     * @brief A subnormal field is a valid double and is not rejected.
     * @details strtod() reports the gradual underflow of a subnormal result
     *          by setting errno to ERANGE, which GEO::String::from_string()
     *          (used by field_as_double() before) reported as a conversion
     *          failure. Hexahedral mappings do write such coordinates: the
     *          file i13u_s6_MC.hexex carries a vertex whose x is the smallest
     *          positive subnormal, 4.9406564584124654e-324.
     */
    TEST(LineStream, DoubleReadsSubnormalFields) {
        EXPECT_DOUBLE_EQ(double_of("4.9406564584124654e-324"), 4.9406564584124654e-324);
        EXPECT_DOUBLE_EQ(double_of("-4.9406564584124654e-324"), -4.9406564584124654e-324);
        EXPECT_DOUBLE_EQ(double_of("1e-320"), 1e-320);
        EXPECT_DOUBLE_EQ(double_of("2.2250738585072014e-308"), 2.2250738585072014e-308);
        EXPECT_DOUBLE_EQ(double_of("0"), 0.0);
        EXPECT_DOUBLE_EQ(double_of("1e-400"), 0.0);
        EXPECT_DOUBLE_EQ(double_of("-4503599627370495.5"), -4503599627370495.5);
    }

    /**
     * @brief field_as_double() still rejects malformed and overflowing fields.
     */
    TEST(LineStream, DoubleRejectsInvalidFields) {
        for(const auto& field : {
            "abc", "1.2.3", "1e400", "-1e400", "1e", "0x"
        }) {
            SCOPED_TRACE(field);
            EXPECT_TRUE(double_rejects(field));
        }
    }

    /**
     * @brief A value below the normal range keeps its subnormal value instead
     *        of collapsing to zero.
     * @details The power of ten is applied in chunks (see scale_by_pow10()),
     *          so 1 * 10^-340 is the subnormal 1e-340-ish value rather than
     *          1e16 * (10^-340 == 0).
     */
    TEST(LineStream, ApproximateDoubleKeepsSubnormalValues) {
        EXPECT_DOUBLE_EQ(
            appro_double_of("4.9406564584124654e-324"), 4.9406564584124654e-324);
        EXPECT_NEAR(
            appro_double_of("1", "1" + std::string(320, '0')) / 1e-320, 1.0, 1e-10);
        EXPECT_NEAR(
            appro_double_of("5" + std::string(16, '0'), "1" + std::string(340, '0')) / 4.9406564584124654e-324,
            1.0, 1e-10);
    }
}
