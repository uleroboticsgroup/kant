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

#ifndef KANT_PDDL_OBJECT_DAO_HPP
#define KANT_PDDL_OBJECT_DAO_HPP

#include <string>

#include "kant_dto/pddl_object_dto.hpp"

namespace kant {
namespace dao {
namespace dao_interface {

class PddlObjectDao {
public:
  virtual std::shared_ptr<kant::dto::PddlObjectDto>
  get(std::string object_name) = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> get_all() = 0;

  virtual bool save(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool update(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool
  save_update(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;

  virtual bool
  delete_one(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool delete_all() = 0;
};

} // namespace dao_interface
} // namespace dao
} // namespace kant

#endif
