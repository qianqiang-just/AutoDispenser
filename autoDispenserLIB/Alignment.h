#pragma once
#include <string>
#include "open3d\Open3D.h"
#include <time.h>


namespace my
{
	class Alignment
	{
	public:
		Alignment(double voxel_size = 0.08);
		~Alignment();

		void setVoxelSize(double size);

		bool isVisualize = false;//是否绘图
		bool isVerbose = true;//是否输出详细提示文本

		double voxel_size = 0.08;  // 下采样体素大小
		int min_pt = 50;              // 分割时每个簇的最小点数
		double eps = 0.3;              // 分割时密度参数

		std::vector<open3d::geometry::PointCloud> pcdsOriginal;

		std::vector<Eigen::Matrix4d> init_transformation;//初始位姿
		std::vector<Eigen::Matrix4d> finalTransformation;  //优化后的位姿

		void getMatrice(std::string short1Path, std::string long2Path, std::string short3Path, std::string Long4Path);

	private:
		std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pPcds;//运行过程中的点云，其数据会随着处理过程不断发生变化
		

		
		double overlapPadding = 1.0;      //获取重叠区域的padding参数
		double max_correspondence_distance_coarse = voxel_size * 15;  // 粗糙ICP配准的对应点最大距离
		double max_correspondence_distance_fine = voxel_size * 1.5;   // 精细ICP配准的对应点最大距离
		int nPcds = 0;//pPcds的size
		//std::vector<std::pair<int, int>> framePairs = { {0,1},{1,2},{2,3},{3,1} };
		open3d::pipelines::registration::PoseGraph poseGraph;

		time_t start, end;//计时代码段开头结尾时间变量
		std::vector<std::string> frameName = { "短边1","长边2" ,"短边3" ,"长边4" };

		void readPLYData(std::string filePath);
		void preprocessData(std::shared_ptr<open3d::geometry::PointCloud> & pPcd, bool isDownsampling, bool isRemovingOutLiner, bool isRemovingUnrelated);

		//获取重叠点云
		void loadOverlap();
		std::shared_ptr<open3d::geometry::PointCloud> getOverlap(int firstPcdIndex, int secondPcdIndex);
		
		void fullRegistration();
		Eigen::Matrix4d pairwiseRegistration(int i,Eigen::Matrix4d odometry);


	};
}


