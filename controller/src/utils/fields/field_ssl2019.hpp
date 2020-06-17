#ifndef FIELD_SSL2019_HPP
#define FIELD_SSL2019_HPP

#include "../field.hpp"

class Field_SSL2019 : public Field {
  public:
    QString name() const {return "Field SSL 2019";}
    float length() const {return 9.00;}
    float width() const {return 6.00;}
    float goalWidth() const {return 1.00;}
    float goalDepth() const {return 0.20;}
    float centerRadius() const {return 0.50;}
    float defenseRadius() const {return 1.00;}
    float defenseStretch() const {return 0.00;}
};

#endif // FIELD_SSL2019_HPP
