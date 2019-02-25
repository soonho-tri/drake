#include "drake/common/symbolic_codegen_util.h"

#include <utility>

namespace drake {

using nlohmann::json;

using std::get;

OrderedDocumentedSymbolDict::OrderedDocumentedSymbolDict(std::string name,
                                                         std::string doc)
    : name_{std::move(name)}, doc_{std::move(doc)} {}

OrderedDocumentedSymbolDict::OrderedDocumentedSymbolDict(
    std::string name, std::string doc,
    const std::vector<std::tuple<std::string, std::string, std::string>>&
        members)
    : OrderedDocumentedSymbolDict(std::move(name), std::move(doc)) {
  for (const auto& member : members) {
    AddMember(get<0>(member), get<1>(member), get<2>(member));
  }
}

void OrderedDocumentedSymbolDict::AddMember(const std::string& name,
                                            const std::string& unit,
                                            const std::string& doc) {
  member_names_.push_back(name);
  member_units_.push_back(unit);
  member_docs_.push_back(doc);

  map_.emplace(name, symbolic::Variable{name});
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

symbolic::Variable OrderedDocumentedSymbolDict::operator[](
    const std::string& name) const {
  return map_.at(name);
}

// NOLINTNEXTLINE(runtime/references)
void to_json(nlohmann::json& j, const OrderedDocumentedSymbolDict& dict) {
  j = dict.ToJson();
}

}  // namespace drake
