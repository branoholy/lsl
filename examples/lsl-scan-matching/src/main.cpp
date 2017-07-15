#include <iostream>

#include "lsl/containers/pointcloud.hpp"

#include "lsl/io/carmenstream.hpp"
#include "lsl/io/imagestream.hpp"

typedef lsl::containers::PointCloud2d PointCloud;

int main()
{
	std::cout << "LSL Scan Matching" << std::endl;

	std::size_t paramSkipScans = 19; // 275, 340, 344, 347
	std::size_t paramLoadScans = 2; // 2

	std::string logPath = "/home/brano/prog/registration-tester/etc/datasets/intel/intel.log";
	std::string mapPath = "map.png";

	lsl::io::CARMENStream<PointCloud> inputStream(logPath, 60);
	inputStream.zoom = 50;
	inputStream.skipCommands(paramSkipScans);

	auto clouds = inputStream.loadAll(paramLoadScans);

	lsl::io::ImageStream<PointCloud> outputStream(mapPath);
	outputStream.padding = 100;
	outputStream.saveAll(clouds);

	return 0;
}
