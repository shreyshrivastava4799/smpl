////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_CSV_PARSER_H
#define SMPL_CSV_PARSER_H

#include <stdio.h>
#include <istream>
#include <string>
#include <vector>

namespace smpl {

class CSVParser
{
public:

    bool parseStream(std::istream& is, bool has_header = false);

    size_t fieldCount() const { return m_field_count; }

    size_t recordCount() const {
        if (m_has_header) {
            return m_fields.size() / m_field_count - 1;
        } else {
            return m_fields.size() / m_field_count;
        }
    }

    size_t totalFieldCount() const { return m_fields.size() - m_field_count; }

    bool hasHeader() const { return m_has_header; }
    const std::string& nameAt(size_t ni) const;

    const std::string& fieldAt(size_t ri, size_t fi) const;

private:

    bool m_has_header;
    size_t m_field_count;

    /// storage for all header names and fields
    std::vector<std::string> m_fields;

    bool parseRecord(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseField(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& field);

    bool parseEscaped(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& text);

    bool parseNonEscaped(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& text);

    bool parseOptionalHeader(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseHeader(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseTextData(
        std::istream& s,
        std::istream::pos_type& pos,
        std::istream::char_type& c);

    bool parseDoubleQuote(std::istream& s, std::istream::pos_type& pos);

    bool parseDoubleQuote2(std::istream& s, std::istream::pos_type& pos);

    bool parseComma(std::istream& s, std::istream::pos_type& pos);

    bool parseCR(std::istream& s, std::istream::pos_type& pos);

    bool parseLF(std::istream& s, std::istream::pos_type& pos);

    bool parseCRLF(std::istream& s, std::istream::pos_type& pos);

    bool parseName(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& name);
};

} // namespace smpl

#endif
