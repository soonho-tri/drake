#include "drake/common/symbolic_codegen_util.h"

#include <utility>

namespace drake {

using nlohmann::json;

OrderedDocumentedSymbolDict::OrderedDocumentedSymbolDict(std::string name,
                                                         std::string doc)
    : name_{std::move(name)}, doc_{std::move(doc)} {}

void OrderedDocumentedSymbolDict::AddMember(const std::string& name,
                                            const std::string& unit,
                                            const std::string& doc,
                                            symbolic::Variable::Type type) {
  member_names_.push_back(name);
  member_units_.push_back(unit);
  member_docs_.push_back(doc);
  member_types_.push_back(type);
}

json OrderedDocumentedSymbolDict::ToJson() const {
  json members;
  for (size_t i = 0; i < member_names_.size(); ++i) {
    json member;
    member["name"] = member_names_[i];
    member["units"] = member_units_[i];
    member["doc_string"] = member_docs_[i];
    members.push_back(member);
  }
  json data;
  data[name_] = members;
  return data;
}

}  // namespace drake
