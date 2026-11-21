//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LINE_STREAM_H
#define GEOLIO_LINE_STREAM_H

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <geogram/basic/numeric.h>
#include <geogram/basic/string.h>
#include <cassert>

/**
 * This file is based on <geogram/basic/line_stream.h>
 *
 * \note Modification vs. the original geogram implementation:
 * - The original class stored the current line in a fixed size buffer
 *   (char line_[MAX_LINE_LEN], 65535 bytes), so lines longer than that
 *   were truncated and the leftover part of the line was returned as a
 *   bogus extra line by the following get_line() call. In this version
 *   the line buffer is a dynamically growing std::string (see read_line()),
 *   so lines of arbitrary length are read completely.
 * - get_bracket_keyword() was added: it consumes a keyword(...)
 *   construct at the start of the current line, e.g.
 *   get_bracket_keyword("(") on "Elements(223852, ...)" returns the
 *   whitespace-trimmed keyword "Elements" and keeps the content inside
 *   the parentheses ("223852, ...") as the new line. It throws
 *   std::logic_error when the token is empty, and returns an empty
 *   string when the token is not found in the line.
 * - field_as_appro_double() was added: it converts a field written as a
 *   rational number "p/q" with arbitrarily large integers (as produced
 *   by Ansys AEDT files) into an approximate double, e.g.
 *   "-9910370759100826714601236637/3271414488397938850655109120".
 *   The reduction is done on the significands and the powers of ten
 *   separately, so that a quotient is read correctly even when its
 *   numerator or denominator (or both) do not fit in long double, e.g.
 *   the 300+ digits rationals written by hexahedral integer grid
 *   mappings whose quotient is an ordinary grid coordinate.
 */
namespace geolio
{

    /**
     * \brief Reads an ASCII file line per line
     * \details LineInput reads an ASCII file line by line and splits
     * the line into a list of white space separated fields that can be
     * accessed individually or converted to numeric values.
     *
     * Functions field_as_int() and field_as_double() throw exceptions when
     * they cannot convert a field to a integer or floating point value, so
     * it is safe to wrap the LineNumber usage in a try / catch block as
     * follows:
     *
     * \code
     * try {
     *     LineInput in(filename);
     *     while( !in.eof() && in.get_line() ) {
     *         in.get_fields();
     *         double d = in.field_as_double(2);
     *     }
     * }
     * catch(const std::logic_error& ex) {
     *     std::cerr << "Got an error: " << ex.what() << std::endl;
     * }
     * \endcode
     */
    class LineInput {
    public:
    /**
     * \brief Creates a new line reader from a file
     * \details This open the file \p filename for reading and prepares to
     * read it line by line. If the file could not be opened, OK() will
     * return false;
     * \param[in] filename the name of the file to read
     */
    LineInput(const std::string& filename);

    /**
     * \brief Destroys the line reader
     * \details This closes the current input file.
     */
    ~LineInput();

    /**
     * \brief Checks if the line reader is ready to read.
     */
    [[nodiscard]] bool OK() const {
        return ok_;
    }

    /**
     * \brief Checks if line reader has reached the end of the input stream
     * \retval true if the stream is at end
     * \retval false otherwise
     */
    [[nodiscard]] bool eof() const {
        return feof(F_) ? true : false;
    }

    /**
     * \brief Reads a new line
     * \details Reads a new line from the input stream. Function
     * get_fields() must be called if you need to access to individual
     * fields in the line with field() and its typed variants.
     * \retval true if a line could be read
     * \retval false otherwise.
     */
    bool get_line();

    /**
     * \brief Gets the number of fields in the current line
     * \details Function get_fields() must be called once after get_line()
     * before calling this function, otherwise the result is undefined.
     * \return the number of fields in the current line
     */
    [[nodiscard]] GEO::index_t nb_fields() const {
        return static_cast<GEO::index_t>(field_.size());
    }

    /**
     * \brief Returns the current line number
     * \details If no line has been read so far, line_number() returns 0.
     */
    [[nodiscard]] size_t line_number() const {
        return line_num_;
    }

    /**
     * \brief Gets a line field as a modifiable string
     * \details The function returns the field at index \p i. Function
     * get_fields() must be called once after get_line() before calling
     * this function, otherwise the result is undefined.
     * \param[in] i the index of the field
     * \return the modifiable pointer to field string at index \p i
     */
    [[nodiscard]] char* field(const GEO::index_t i) {
        assert(i < nb_fields());
        return field_[i];
    }

    /**
     * \brief Gets a line field as a non-modifiable string
     * \details The function returns the field at index \p i. Function
     * get_fields() must be called once after get_line() before calling
     * this function, otherwise the result is undefined.
     * \param[in] i the index of the field
     * \return the const pointer to field string at index \p i
     */
    [[nodiscard]] const char* field(const GEO::index_t i) const {
        assert(i < nb_fields());
        return field_[i];
    }

    /**
     * \brief Gets a line field as an integer.
     * \details The function returns the field at index \p i converted to
     * an integer. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \param[in] i the index of the field
     * \return the integer value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to an
     * integer value
     */
    [[nodiscard]] GEO::signed_index_t field_as_int(GEO::index_t i) const {
        GEO::signed_index_t result = 0;
        if(!GEO::String::from_string(field(i), result)) {
            conversion_error(i, "integer");
        }
        return result;
    }

    /**
     * \brief Gets a line field as an unsigned integer.
     * \details The function returns the field at index \p i converted to
     * an unsigned integer. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \param[in] i the index of the field
     * \return the unsigned integer value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to an
     * unsigned integer value
     */
    [[nodiscard]] GEO::index_t field_as_uint(GEO::index_t i) const {
        GEO::index_t result = 0;
        if(!GEO::String::from_string(field(i), result)) {
            conversion_error(i, "unsigned integer");
        }
        return result;
    }

    /**
     * \brief Gets a line field as a double.
     * \details The function returns the field at index \p i converted to
     * a double. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \details Unlike GEO::String::from_string(), this function accepts a
     * subnormal value. strtod() reports a gradual underflow (e.g. the
     * coordinate "4.9406564584124654e-324" written by hexahedral
     * mappings) by setting errno to ERANGE, but the value it returns is a
     * perfectly valid double; only an overflow, whose value is infinite,
     * is an error.
     * \param[in] i the index of the field
     * \return the floating point value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to a
     * floating point value
     */
    [[nodiscard]] double field_as_double(GEO::index_t i) const {
        const char* begin = field(i);
        errno = 0;
        char* end = nullptr;
        const double result = strtod(begin, &end);
        if(end == begin || *end != '\0') {
            conversion_error(i, "floating point");
        }
        if(errno == ERANGE && !std::isfinite(result)) {
            conversion_error(i, "floating point");
        }
        return result;
    }

    /**
     * \brief Gets a line field as an approximate double.
     * \details The function returns the field at index \p i converted to
     * a double. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \details A field written as a rational number "p/q" with
     * arbitrarily large integers (e.g. "-9910370759100826714601236637/
     * 3271414488397938850655109120", as produced by Ansys AEDT files) is
     * converted to an approximate double. Since the numerator and the
     * denominator may individually exceed the range of long double (on
     * Apple Silicon long double is 64 bit, that is the same range as
     * double, and hexahedral integer grid mappings emit rationals with
     * more than 300 digits) while their quotient is an ordinary number,
     * each part is reduced to a significand of at most 19 significant
     * digits plus a power of ten (see parse_decimal_field()), and the two
     * powers of ten are recombined instead of dividing two converted
     * parts. Fields that do not contain a '/' are converted the same way,
     * so they are accepted with any number of digits as well. The result
     * carries the accuracy of a double (a few ulps), which is what
     * "approximate" means here; a result below the normal range is rounded
     * to a subnormal rather than to zero (see scale_by_pow10()).
     * \param[in] i the index of the field
     * \return the floating point value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to a
     * floating point value, including when the value itself overflows the
     * range of double.
     */
    [[nodiscard]] double field_as_appro_double(GEO::index_t i) const {
        const char* begin = field(i);
        const char* const end = begin + strlen(begin);
        const char* slash = strchr(begin, '/');
        if(slash == nullptr) {
            bool negative = false;
            double significand = 0.0;
            long long exponent = 0;
            if(!parse_decimal_field(begin, end, negative, significand, exponent)) {
                conversion_error(i, "floating point");
            }
            const double value = scale_by_pow10(significand, exponent);
            // Overflowing the range of double is an error, unlike an
            // intermediate part of a rational that does not fit in a double.
            if(!std::isfinite(value)) {
                conversion_error(i, "floating point");
            }
            return negative ? -value : value;
        }

        bool num_negative = false;
        bool den_negative = false;
        double num_significand = 0.0;
        double den_significand = 0.0;
        long long num_exponent = 0;
        long long den_exponent = 0;
        if(!parse_decimal_field(begin, slash, num_negative, num_significand, num_exponent) ||
           !parse_decimal_field(slash + 1, end, den_negative, den_significand, den_exponent) ||
           den_significand == 0.0) {
            conversion_error(i, "floating point");
        }

        // The two parts may be far outside the range of double even though
        // their quotient is not (e.g. 1e356/1e355), so the powers of ten
        // are recombined here rather than by dividing two converted parts.
        const double ratio = num_significand / den_significand;
        const double value = scale_by_pow10(ratio, num_exponent - den_exponent);
        // Overflowing the range of double is an error, unlike an intermediate
        // part of the quotient that does not fit in a double.
        if(!std::isfinite(value)) {
            conversion_error(i, "floating point");
        }
        return (num_negative != den_negative) ? -value : value;
    }

    /**
     * \brief Compares a field with a string.
     * \details The function compares the field at index \p i with string
     * \p s and returns \c true if they are equal. Function get_fields()
     * must be called once after get_line() before calling this function,
     * otherwise the result is undefined.
     * \param[in] i the index of the field
     * \param[in] s the string to compare the field to
     * \retval true if field at index \p i equals string \p s
     * \retval false otherwise
     */
    bool field_matches(GEO::index_t i, const char* s) const {
        return strcmp(field(i), s) == 0;
    }

    /**
     * \brief Splits the current line into fields.
     * \details The function uses \p separators to split the
     * current line into individual fields that can be accessed
     * by field() and its typed variants.
     * \param[in] separators a string that contains all
     *  the characters considered as separators.
     * \see field()
     */
    void get_fields(const char* separators = " \t\r\n");

    /**
     * \brief Gets the keyword of a keyword(...) construct at the start
     * of the current line.
     * \details Extracts the keyword before the first occurrence of
     * \p token (typically '('), removes it from the current line
     * together with the token, and keeps the content between the token
     * and its matching closing delimiter (typically ')') as the new
     * current line. The returned keyword is trimmed of leading and
     * trailing whitespace.
     * \param[in] token the opening delimiter, e.g. "("
     * \return the trimmed keyword that was before \p token, or an empty
     * string if \p token is not present in the current line
     * \exception std::logic_error if \p token is empty
     * \note This function modifies the current line, so any field
     * pointer previously returned by field() is invalidated. It is
     * meant to be used on the raw line, before calling get_fields().
     * \code
     * // line_ == "Elements(223852, 110460, 4, 3)"
     * std::string kw = in.get_bracket_keyword("(");
     * // kw == "Elements" and line_ == "223852, 110460, 4, 3"
     * \endcode
     */
    [[nodiscard]] std::string get_bracket_keyword(const std::string& token);

    /**
     * \brief Gets the current line.
     * \details If get_fields() was called, then an end-of-string
     *  marker '\0' is present at the end of the first field.
     * \return a const pointer to the internal buffer that stores
     *  the current line
     */
    [[nodiscard]] const char* current_line() const {
        return line_.c_str();
    }

    private:
    /**
     * \brief Reads a full line of arbitrary length from the input file.
     * \details Reads characters until the end of the line or the end of the
     * file, growing \p out as needed, so lines longer than any fixed buffer
     * are read completely.
     * \param[out] out the string receiving the line
     * \retval true if a line could be read
     * \retval false if the end of the file was reached without reading
     * any character
     */
    bool read_line(std::string& out) const;

    /**
     * \brief Multiplies a number by a power of ten.
     * \details The power is applied in chunks of 10^300, so that only the
     * final result can overflow or underflow and not an intermediate
     * factor. This keeps a subnormal result meaningful: 4.9406564584124654e16
     * times 10^-340 is 4.9406564584124654e-324, while 10^-340 itself
     * already underflows to zero.
     * \param[in] value the number to scale
     * \param[in] exponent the power of ten to scale it by
     * \return \p value times 10^\p exponent, possibly infinite or zero
     */
    static double scale_by_pow10(double value, long long exponent) {
        // 10^300 is a double, and there is room for one more factor of it
        // before the result would have to overflow.
        constexpr long long chunk = 300;
        while(exponent > chunk && value != 0.0 && std::isfinite(value)) {
            value *= 1e300;
            exponent -= chunk;
        }
        while(exponent < -chunk && value != 0.0) {
            value *= 1e-300;
            exponent += chunk;
        }
        return value * std::pow(10.0, double(exponent));
    }

    /**
     * \brief Converts a decimal literal into a significand and a power of ten.
     * \details The characters in [\p begin, \p end) are read as a decimal
     * number (sign, digits, optional decimal point, optional exponent, and
     * surrounding whitespace) and written back as \p significand times
     * 10^(\p exponent). Only the 19 leading significant digits are kept,
     * which is enough to round a double correctly, and every other digit is
     * folded into the exponent. A literal of any length can therefore be
     * converted, including literals whose value or whose digit string does
     * not fit in long double / in an integer type.
     * \param[in] begin the first character of the literal
     * \param[in] end one past the last character of the literal
     * \param[out] negative true if the literal is negative
     * \param[out] significand the leading significant digits, as a double
     * \param[out] exponent the power of ten to apply to \p significand
     * \retval true if the whole of [\p begin, \p end) is a decimal number
     * \retval false otherwise
     */
    static bool parse_decimal_field(
        const char* begin, const char* end,
        bool& negative, double& significand, long long& exponent
    ) {
        const auto is_space = [](char c) {
            return c == ' ' || c == '\t' || c == '\n' || c == '\r' ||
                   c == '\f' || c == '\v';
        };
        const auto is_digit = [](char c) {
            return c >= '0' && c <= '9';
        };

        while(begin != end && is_space(*begin)) {
            ++begin;
        }
        while(end != begin && is_space(*(end - 1))) {
            --end;
        }
        if(begin == end) {
            return false;
        }

        negative = false;
        if(*begin == '+' || *begin == '-') {
            negative = (*begin == '-');
            ++begin;
        }

        // The value is significand * 10^exponent, where significand holds the
        // first `nb_kept` significant digits, i.e. the digits from the first
        // non-zero one on. `nb_digits` counts all the digits of the literal
        // (leading zeros included) and `nb_int_digits` the ones before the
        // decimal point, so that the decimal point contributes
        // nb_int_digits - nb_digits while the digits that do not fit in
        // significand contribute nb_significant_digits - nb_kept:
        // value = significand * 10^(nb_int_digits - nb_digits
        //                            + nb_significant_digits - nb_kept)
        constexpr long long max_kept = 19;
        char kept[max_kept];
        long long nb_kept = 0;
        long long nb_digits = 0;
        long long nb_int_digits = 0;
        long long nb_significant_digits = 0;
        bool started = false;

        while(begin != end && is_digit(*begin)) { // Integer part.
            const char digit = *begin++;
            ++nb_int_digits;
            ++nb_digits;
            if(!started) {
                if(digit == '0') {
                    continue;
                }
                started = true;
            }
            ++nb_significant_digits;
            if(nb_kept < max_kept) {
                kept[nb_kept++] = digit;
            }
        }
        if(begin != end && *begin == '.') { // Fractional part.
            ++begin;
            while(begin != end && is_digit(*begin)) {
                const char digit = *begin++;
                ++nb_digits;
                if(!started) {
                    if(digit == '0') {
                        continue;
                    }
                    started = true;
                }
                ++nb_significant_digits;
                if(nb_kept < max_kept) {
                    kept[nb_kept++] = digit;
                }
            }
        }
        long long exp10 = 0;
        if(begin != end && (*begin == 'e' || *begin == 'E')) { // Exponent part.
            ++begin;
            bool exp_negative = false;
            if(begin != end && (*begin == '+' || *begin == '-')) {
                exp_negative = (*begin == '-');
                ++begin;
            }
            if(begin == end || !is_digit(*begin)) {
                return false;
            }
            long long exp_value = 0;
            while(begin != end && is_digit(*begin)) {
                // The exponent saturates long before it could overflow, and
                // any value beyond the range of double is saturated anyway.
                if(exp_value < 1000000) {
                    exp_value = exp_value * 10 + (*begin - '0');
                }
                ++begin;
            }
            exp10 = exp_negative ? -exp_value : exp_value;
        }
        if(begin != end) { // Trailing characters.
            return false;
        }
        if(nb_digits == 0) { // No digit at all, e.g. "" or "e10".
            return false;
        }
        if(!started) { // A zero mantissa, e.g. "0", "-0.000" or "0e10".
            significand = 0.0;
            exponent = 0;
            return true;
        }

        // At most max_kept == 19 decimal digits were kept, which always fits
        // in an unsigned 64 bit integer, so the significand is accumulated
        // exactly and rounded only once, when it is converted to a double.
        unsigned long long exact = 0;
        for(long long d = 0; d < nb_kept; ++d) {
            exact = exact * 10ULL + static_cast<unsigned long long>(kept[d] - '0');
        }
        significand = static_cast<double>(exact);
        exponent = nb_int_digits - nb_digits +
                   (nb_significant_digits - nb_kept) + exp10;
        return true;
    }

    /**
     * \brief Throws a conversion error.
     * \details This function is called by field_as_int() and
     * field_as_double() when the field \p index cannot be converted to
     * the desired type \p type.
     * \param[in] index index of the erroneous field.
     * \param[in] type the expected type.
     */
    GEO_NORETURN_DECL void conversion_error(
        GEO::index_t index, const char* type
    ) const GEO_NORETURN ;

    FILE* F_;
    std::string file_name_;
    size_t line_num_;
    std::string line_;
    std::vector<char*> field_;
    bool ok_;
    };
}

#endif //GEOLIO_LINE_STREAM_H
