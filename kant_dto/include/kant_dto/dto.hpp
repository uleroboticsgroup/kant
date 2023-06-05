// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PDDL_DTO_HPP
#define PDDL_DTO_HPP

#include <memory>
#include <string>

namespace kant {
namespace dto {

class Dto {

public:
  virtual ~Dto() {}
  virtual std::string to_string() = 0;
  virtual bool equals(std::shared_ptr<Dto> dto) = 0;
};

} // namespace dto
} // namespace kant

#endif