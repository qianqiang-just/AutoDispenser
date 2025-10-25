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

		bool isVisualize = false;//�Ƿ��ͼ
		bool isVerbose = true;//�Ƿ������ϸ��ʾ�ı�

		double voxel_size = 0.08;  // �²������ش�С
		int min_pt = 50;              // �ָ�ʱÿ���ص���С����
		double eps = 0.3;              // �ָ�ʱ�ܶȲ���

		std::vector<open3d::geometry::PointCloud> pcdsOriginal;

		std::vector<Eigen::Matrix4d> init_transformation;//��ʼλ��
		std::vector<Eigen::Matrix4d> finalTransformation;  //�Ż����λ��

		void getMatrice(std::string short1Path, std::string long2Path, std::string short3Path, std::string Long4Path);

	private:
		std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pPcds;//���й����еĵ��ƣ������ݻ����Ŵ�����̲��Ϸ����仯
		

		
		double overlapPadding = 1.0;      //��ȡ�ص������padding����
		double max_correspondence_distance_coarse = voxel_size * 15;  // �ֲ�ICP��׼�Ķ�Ӧ��������
		double max_correspondence_distance_fine = voxel_size * 1.5;   // ��ϸICP��׼�Ķ�Ӧ��������
		int nPcds = 0;//pPcds��size
		//std::vector<std::pair<int, int>> framePairs = { {0,1},{1,2},{2,3},{3,1} };
		open3d::pipelines::registration::PoseGraph poseGraph;

		time_t start, end;//��ʱ����ο�ͷ��βʱ�����
		std::vector<std::string> frameName = { "�̱�1","����2" ,"�̱�3" ,"����4" };

		void readPLYData(std::string filePath);
		void preprocessData(std::shared_ptr<open3d::geometry::PointCloud> & pPcd, bool isDownsampling, bool isRemovingOutLiner, bool isRemovingUnrelated);

		//��ȡ�ص�����
		void loadOverlap();
		std::shared_ptr<open3d::geometry::PointCloud> getOverlap(int firstPcdIndex, int secondPcdIndex);
		
		void fullRegistration();
		Eigen::Matrix4d pairwiseRegistration(int i,Eigen::Matrix4d odometry);


	};
}


