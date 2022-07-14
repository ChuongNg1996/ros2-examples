/*
LINK: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html
*/

#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  // We’re creating an abstract class called "RegularPolygon"
  class RegularPolygon
  {
    /* One thing to notice is the presence of the initialize method. With pluginlib, 
    a constructor without parameters is required for classes so, if any parameters are 
    required, we use the initialize method to initialize the object*/
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP