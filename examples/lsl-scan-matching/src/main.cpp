#include <functional>
#include <iostream>
#include <vector>

#include "lsl/containers/pointcloud.hpp"

#include "lsl/io/imagestream.hpp"
#include "lsl/io/simplestream.hpp"

typedef lsl::containers::PointCloud2d PointCloud;
typedef std::function<void (PointCloud&)> transformFn;

void saveTransformedClouds(const std::string& filePath, std::vector<PointCloud> clouds, transformFn transformCloud = [] (PointCloud&) {});

int main()
{
	std::cout << "LSL Scan Matching" << std::endl;

	// std::size_t paramSkipScans = 19; // 275, 340, 344, 347
	std::size_t paramLoadScans = 10; // 2

	std::string logPath = "/home/brano/prog/school/dpslam/trunk/loop5.log";

	lsl::io::SimpleStream<PointCloud> inputStream(logPath);
	inputStream.zoom = 50;
	// inputStream.skipCommands(paramSkipScans);

	auto clouds = inputStream.loadAll(paramLoadScans);

	auto transformToOdom = [] (PointCloud& cloud) { cloud.transformToLocation(cloud.getOdomLocation()); };
	auto transformToCorrected = [] (PointCloud& cloud) { cloud.transformToLocation(cloud.getCorrectedLocation()); };

	saveTransformedClouds("map-0-orig.png", clouds);
	saveTransformedClouds("map-1-odom.png", clouds, transformToOdom);

	return 0;
}

void saveTransformedClouds(const std::string& filePath, std::vector<PointCloud> clouds, transformFn transformCloud)
{
	for(auto& cloud : clouds)
	{
		transformCloud(cloud);
	}

	lsl::io::ImageStream<PointCloud> outputStream(filePath);
	outputStream.padding = 100;
	outputStream.saveAll(clouds);
}
