#ifndef FIELD_HPP
#define FIELD_HPP

#include<QString>

class Field {
  public:
      virtual ~Field() {}
      virtual QString name() const = 0;

      // Field dimensions
      virtual float length() const = 0;
      virtual float width() const = 0;

      // Goal dimensions
      virtual float goalWidth() const = 0;
      virtual float goalDepth() const = 0;

      // Center dimensions
      virtual float centerRadius() const = 0;

      // Defense area dimensions
      virtual float defenseRadius() const = 0;
      virtual float defenseStretch() const = 0;
      float defenseLength() const {return 2*defenseRadius()+defenseStretch();}
      float defenseWidth() const {return defenseRadius();}

      // Min/max X and Y
      float minX() const {return -length()/2;}
      float maxX() const {return length()/2;}
      float minY() const {return -width()/2;}
      float maxY() const {return width()/2;}
};

#endif // FIELD_HPP
