#include <functional>
#include <iostream>
#include <vector>

#include "lsl/containers/pointcloud.hpp"
#include "lsl/registration/llr.hpp"

#include "lsl/io/imagestream.hpp"
#include "lsl/io/simplestream.hpp"

typedef lsl::containers::PointCloud2d PointCloud;
typedef std::function<void (PointCloud&)> transformFn;

void saveTransformedClouds(const std::string& filePath, std::vector<PointCloud> clouds, transformFn transformCloud = [] (PointCloud&) {});
void saveLines(const std::string& filePath, const std::vector<PointCloud>& clouds, const std::vector<std::vector<lsl::geom::LidarLine2>>& lines);

int main()
{
	std::cout << "LSL Scan Matching" << std::endl;

	bool paramSaveLines = true;
	std::size_t paramSkipScans = 0; // 275, 340, 344, 347
	std::size_t paramLoadScans = 100; // 2

	std::string logPath = "/home/brano/prog/school/dpslam/trunk/loop5.log";
	PointCloud::Transformation detectionTransformation = PointCloud::createTransformation(PointCloud::Location(0, 0, lsl::utils::MathUtils::PI));

	lsl::io::SimpleStream<PointCloud> inputStream;
	inputStream.zoom = 50;
	// inputStream.skipCommands(paramSkipScans);

	auto clouds = inputStream.loadAll(logPath, paramLoadScans);

	auto transformToOdom = [] (PointCloud& cloud) { cloud.transformToLocation(cloud.getOdomLocation()); };
	auto transformToCorrected = [] (PointCloud& cloud) { cloud.transformToLocation(cloud.getCorrectedLocation()); };

	saveTransformedClouds("results/map-0-orig.png", clouds);
	saveTransformedClouds("results/map-1-odom.png", clouds, transformToOdom);

	lsl::registration::LLR llr;
	llr.setDetectionTransformation(detectionTransformation);
	llr.getRansac().set(50, 2, 4, 2.1);
	llr.getSplitMerge().set(4, 2.1);
	llr.setMaxDiffL(80);
	// llr.setMaxDiffAlpha(0.7);
	llr.setMaxIterations(100);
	llr.setMaxTries(30);
	// llr.setMinErrorDiff(0.01);
	llr.setMinFinalError(0.0001);
	llr.getGammas() = {0.7, 0.6, 0.00001};

	clouds.front().getCorrectedLocation() = clouds.front().getOdomLocation();
	for(std::size_t i = 1; i < clouds.size(); i++)
	{
		const auto& target = clouds.at(i - 1);
		auto& source = clouds.at(i);

		auto sourceT = source;
		std::vector<lsl::geom::LidarLine2> sourceLinesT;

		auto targetLocation = PointCloud::createTransformation(target.getOdomLocation());
		auto sourceLocation = PointCloud::createTransformation(source.getOdomLocation());
		auto guess = targetLocation.inverse() * sourceLocation;

		llr.setTarget(target);
		llr.setSource(source);

		std::cout << llr.getTargetLines() << std::endl << std::endl;
		std::cout << llr.getSourceLines() << std::endl << std::endl;

		std::string linesPath;
		if(paramSaveLines)
		{
			linesPath = "results/lines/lines-" + std::to_string(i - 1) + 'x' + std::to_string(i);
			saveLines(linesPath + "-0-orig.png", {target, source}, {llr.getTargetLines(), llr.getSourceLines()});

			sourceT.transform(guess);
			sourceLinesT = llr.detectLines(sourceT, llr.getMaxLineCount());
			saveLines(linesPath + "-2-odom.png", {target, sourceT}, {llr.getTargetLines(), sourceLinesT});
		}

		llr.align(guess);
		auto finalLocation = llr.getFinalTransformation();

		if(paramSaveLines)
		{
			sourceT = source;
			sourceT.transform(finalLocation);
			sourceLinesT = llr.detectLines(sourceT, llr.getMaxLineCount());
			saveLines(linesPath + "-3-corrected.png", {target, sourceT}, {llr.getTargetLines(), sourceLinesT});
		}

		auto targetCLocation = PointCloud::createTransformation(target.getCorrectedLocation());
		source.getCorrectedLocation() = PointCloud::createLocation(targetCLocation * finalLocation);

		std::cout << (i - 1 + paramSkipScans) << 'x' << (i + paramSkipScans)
				  << " error: " << llr.getFinalError()
				  << " iters: " << llr.getIterationCount()
				  << " evals: " << llr.getEvaluationCount()
				  << std::endl;
	}

	saveTransformedClouds("results/map-2-corrected.png", clouds, transformToCorrected);

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

void saveLines(const std::string& filePath, const std::vector<PointCloud>& clouds, const std::vector<std::vector<lsl::geom::LidarLine2>>& lines)
{
	lsl::io::ImageStream<PointCloud> outputStream(filePath);
	outputStream.saveAll(clouds, lines);
}
