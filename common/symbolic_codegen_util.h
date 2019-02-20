#pragma once

#include <string>
#include <vector>

#include <inja/inja.hpp>

#include "drake/common/symbolic.h"

namespace drake {

/**
Ordered dictionary of (string_repr, DocumentedSymbol) pairs.
  Allows:
    * iteration with guaranteed ordering
    * access underlying symbol with string representation as the key
    * access underlying symbol with dot notation
  Note:
    String representations of each DocumentedSymbol must be unique.
*/
class OrderedDocumentedSymbolDict {
 public:
  OrderedDocumentedSymbolDict(std::string name, std::string doc);
  void AddMember(
      const std::string& name, const std::string& unit, const std::string& doc,
      symbolic::Variable::Type type = symbolic::Variable::Type::CONTINUOUS);

  nlohmann::json ToJson() const;

 private:
  std::string name_;
  std::string doc_;

  std::vector<std::string> member_names_;
  std::vector<std::string> member_units_;
  std::vector<std::string> member_docs_;
  std::vector<symbolic::Variable::Type> member_types_;
};

}  // namespace drake
