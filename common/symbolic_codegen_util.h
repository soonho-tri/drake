#pragma once

#include <map>
#include <string>
#include <tuple>
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
  OrderedDocumentedSymbolDict(
      std::string name, std::string doc,
      const std::vector<std::tuple<std::string, std::string, std::string>>&
          members);

  void AddMember(const std::string& name, const std::string& unit,
                 const std::string& doc);

  nlohmann::json ToJson() const;

  symbolic::Variable operator[](const std::string& name) const;

 private:
  std::string name_;
  std::string doc_;

  std::vector<std::string> member_names_;
  std::vector<std::string> member_units_;
  std::vector<std::string> member_docs_;

  std::map<std::string, symbolic::Variable> map_;
};

// NOLINTNEXTLINE(runtime/references)
void to_json(nlohmann::json& j, const OrderedDocumentedSymbolDict& dict);

}  // namespace drake
