// person.cpp
#include "DetectCross.hpp"

using detectcross::DetectCross;
using namespace std;

DetectCross::DetectCross(const std::string &name) : m_name(name) {}

std::string DetectCross::sayHello() const {
	return m_name + " says hello to you!";
}

void DetectCross::readJson(void) {

  FILE *fp, *fp2;
    std::cout << "readJson()" << std::endl;

  //if ((fp = fopen("../map/node_link.json", "r")) == NULL) {
  if ((fp = fopen("/home/hj/Documents/project/LDW_IPM_TEST_m8_DB2/map/node_link.json", "r")) == NULL) {    // debugging위한 절대경로
    std::cout << "fopen error" << std::endl;
  }

  char buf[1000000];
  fgets(buf, 1000000, fp);

  cJSON *json = cJSON_Parse((const char *)buf);

  // Parse link data
  cJSON *data = cJSON_GetObjectItem(json, "links");
  link_size = cJSON_GetArraySize(data);
  src = (double *)malloc(sizeof(double) * link_size);
  tag = (double *)malloc(sizeof(double) * link_size);

  std::cout << "dataLength : " << link_size << std::endl;
  
  for (int i = 0; i < link_size; i++) {
      cJSON * dataLink = cJSON_GetArrayItem(data, i);
  
      cJSON * Source = cJSON_GetObjectItem(dataLink, "Source");
      src[i] = atoi(cJSON_Print(Source));
      // std::cout << "Source : " << src[i] << std::endl;

      cJSON * Target = cJSON_GetObjectItem(dataLink, "Target");
      tag[i] = atoi(cJSON_Print(Target));
      // std::cout << "Target : " << tag[i] << std::endl;
  }

  // Parse node data
  cJSON *data_nodes = cJSON_GetObjectItem(json, "nodes");
//   double *nid, *lon, *lat;
	node_size = cJSON_GetArraySize(data_nodes);
  
  nid = (double *)malloc(sizeof(double) * node_size);
  lon = (double *)malloc(sizeof(double) * node_size);
  lat = (double *)malloc(sizeof(double) * node_size);

  std::cout << "dataLength : " << node_size << std::endl;

  for (int i = 0; i < node_size; i++) {
      cJSON * dataNode = cJSON_GetArrayItem(data_nodes, i);
  
      cJSON * Id = cJSON_GetObjectItem(dataNode, "Id");
      nid[i] = atoi(cJSON_Print(Id));
      // std::cout << "Id : " << cJSON_Print(Id) << std::endl;

      cJSON * longitude = cJSON_GetObjectItem(dataNode, "longitude");
      lon[i] = atof(cJSON_Print(longitude));
      // std::cout << "longitude : " << lon[i] << std::endl;

      cJSON * latitude = cJSON_GetObjectItem(dataNode, "latitude");
      lat[i] = atof(cJSON_Print(latitude));
      // std::cout << "latitude : " << lat[i] << std::endl;
  }

  fclose(fp);

// --------------------------------------------------------------------------------------
  // scaling
  for (int i = 0; i < node_size; i++) {
    max_lon = max(max_lon, lon[i]);
    min_lon = min(min_lon, lon[i]);
    max_lat = max(max_lat, lat[i]);
    min_lat = min(min_lat, lat[i]);
  }

  std::cout << "max_lon : " << max_lon << endl;
  std::cout << "min_lon : " << min_lon << endl;
  std::cout << "max_lat : " << max_lat << endl;
  std::cout << "min_lat : " << min_lat << endl;

  for (int i = 0; i < node_size; i++) {
    lon[i] = (lon[i]-min_lon)/(max_lon-min_lon);
    lat[i] = (lat[i]-min_lat)/(max_lat-min_lat);
  }

}

