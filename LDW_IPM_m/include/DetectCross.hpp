// DetectCross.hpp
#ifndef DetectCross_HPP_
#define DetectCross_HPP_

#include <stdlib.h>
#include <iostream>
#include "cJSON.h"
#include <math.h>

namespace detectcross {

class DetectCross {
public:
	explicit DetectCross(const std::string &name);
	std::string sayHello() const;
	void readJson(void);

	double *nid, *lon, *lat;
	double *src, *tag;

	double max_lon = 0;
	double min_lon = INFINITY;
	double max_lat = 0;
	double min_lat = INFINITY;

	int node_size = 0;
	int link_size = 0;

private:
	std::string m_name;
};

} // #end of DetectCross

#endif // DetectCross_HPP_


