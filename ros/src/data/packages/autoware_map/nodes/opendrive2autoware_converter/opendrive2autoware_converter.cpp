
#include <ros/ros.h>
#include <opendrive2autoware_converter/opendrive_loader.h>
#include <opendrive2autoware_converter/map_writer.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opendrive2autoware_converter");



	std::string src_path;
	std::string dst_path;
	double wp_res = 0.5;

	if(argc < 3)
	{
		std::cout << "Too few parameters !" << std::endl;
		std::cout << " Source OpenDRIVE .xodr file name";
		std::cout << " Destination folder for Autoware map files" << std::endl;
		std::cout << " Waypoints resolution (default is 0.5 meters) " << std::endl;
		return 0;
	}
	else
	{
		std::cout << argc << std::endl;

		src_path = std::string(argv[1]);
		dst_path = std::string(argv[2]);
		if(argc > 3)
			wp_res = atof(argv[3]);

		autoware_map::OpenDriveLoader map_loader;
		PlannerHNS::RoadNetwork map;
		map_loader.loadOpenDRIVE(src_path, map);
		autoware_map::MapWriter map_save;
		map_save.writeAutowareMap(dst_path, map);
		//Write autoware maps files
	}

  //  ros::spin();
}
