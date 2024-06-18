#include <unordered_map>

#ifndef ABSTRACTION_TYPE_H
#define ABSTRACTION_TYPE_H

namespace mlmp
{
    namespace abstraction {
        enum AbstractionType {
            Projection=0,
            PriorityOrder=1
        };  

        AbstractionType fromValue(std::string str) {
            static std::unordered_map<std::string, AbstractionType> const table = { 
                {"PROJECTION", AbstractionType::Projection}, 
                {"PRIORITY_ORDER", AbstractionType::PriorityOrder}
            };

            auto it = table.find(str);
            if (it != table.end()) {
                return it->second;
            } else { 
                throw std::invalid_argument("Failed to get abstraction type for " + str);
            }
        }
    }
} 

#endif