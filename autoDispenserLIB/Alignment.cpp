#include "pch.h"
#include "Alignment.h"


my::Alignment::Alignment(double voxel_size)
{
	//�����ʼλ��
	Eigen::Matrix4d matrix;
	//double theta = M_PI / 2;  // // ��ת�ĽǶ��û��ȵı�ʾ����
	//short1
	matrix << -5.11608e-19, 1, 8.67362e-19, 1.88,
			  -1, -1.22481e-18, -4.33681e-19, 167.09,
			  0, 8.67362e-19, 1, 1.63064e-18,
			  0, 0, 0, 1;
	init_transformation.push_back(matrix);

	//long2
	matrix << -0.999971, 0.00446977, -0.0061029, 84.6603,
		      -0.00452183, -0.999953, 0.0085439, 166.112,
		      -0.00606442, 0.00857125, 0.999945, -0.0761714,
		      0, 0, 0, 1;
	init_transformation.push_back(matrix);

	//short3
	//matrix = Eigen::Matrix4d::Identity();
	matrix << 0.000722691, -0.999999, -0.000830143, 82.3594,
			  0.999981, 0.000717599, 0.00611813, -1.69061,
		      -0.00611753, -0.000834548, 0.999981, 1.18393,
			  0, 0, 0, 1;
	init_transformation.push_back(matrix);

	//long4
	matrix << 0.999984, -0.00411278, 0.003985, -0.421192,
		      0.00408689, 0.999971, 0.006485, 0.376519,
		      -0.00401155, -0.0064686, 0.999971, 1.10913,
		      0, 0, 0, 1;
	init_transformation.push_back(matrix);

	setVoxelSize(voxel_size);
}

my::Alignment::~Alignment() {

}





void my::Alignment::loadOverlap()
{	     
	for (int i = 0; i < nPcds; i++)
	{	
		start = clock();
		int preIndex = i - 1;
		int nextIndex = i + 1;
		if (preIndex < 0) 	preIndex = nPcds - 1;
		if (nextIndex > nPcds - 1)   nextIndex = 0;

		std::shared_ptr<open3d::geometry::PointCloud> header = getOverlap(i, preIndex);   // �Լ�ͷ�������ߵ�β
		std::shared_ptr<open3d::geometry::PointCloud> tailer = getOverlap(i, nextIndex);  // �Լ�β����һ����ͷ
		
		if (isVerbose) 	std::cout << frameName[i] << "ͷ������Ԥ����... "  << std::endl;
		preprocessData(header, true, true, true);
		if (isVerbose) 	std::cout << frameName[i] << "β������Ԥ����... " << std::endl;
		preprocessData(tailer, true, true, true);

		*pPcds[i] = *header + *tailer;
		end = clock();
		if (isVerbose) 	std::cout << frameName[i]<<"�ص�������ƴ�С: " << pPcds[i]->points_.size() << std::endl;
		if (isVerbose) 	std::cout << "	�ص�������ȡ��ʱ: " << end - start << "����" << std::endl;

	}	
}

std::shared_ptr<open3d::geometry::PointCloud> my::Alignment::getOverlap(int firstPcdIndex, int secondPcdIndex)
{
	// ��ȡ�������Ƶİ�Χ��
	Eigen::Vector3d padding_bound(overlapPadding, overlapPadding, overlapPadding);
	
	//const auto bbox1 = open3d::geometry::AxisAlignedBoundingBox(pPcds[firstPcdIndex]->GetMinBound() - padding_bound, pPcds[firstPcdIndex]->GetMaxBound() + padding_bound);
	const auto bbox2 = open3d::geometry::AxisAlignedBoundingBox(pPcds[secondPcdIndex]->GetMinBound() - padding_bound, pPcds[secondPcdIndex]->GetMaxBound() + padding_bound);
	// ��ȡ��������֮����ص�����
	return pPcds[firstPcdIndex]->Crop(bbox2);
}

void my::Alignment::fullRegistration()
{
	
	Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();
	poseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odometry));


	for (int i = 0; i < nPcds; i++)
	{
		Eigen::Matrix4d transformationIcp;
		Eigen::Matrix6d informationIcp;
		odometry=pairwiseRegistration(i, odometry);
	}

	start = clock();
	int reference_node = 0;  //��һ������Ϊ�ο�
	auto option = open3d::pipelines::registration::GlobalOptimizationOption(max_correspondence_distance_fine, 0.25, 1.0, reference_node);
	// LM �������Ż�
	GlobalOptimization(poseGraph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(), open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(), option);
	end = clock();
	if (isVerbose) 	std::cout << "	λ��ͼ�Ż���ʱ: " << end - start << "����" << std::endl;
}

Eigen::Matrix4d my::Alignment::pairwiseRegistration(int i,Eigen::Matrix4d odometry)
{
	start = clock();
	int j;
	if (i == 3) j = 0;
	else j = i + 1;
	pPcds[j]->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));

	////�ֲ�ICP��׼ �����Ƿ��б�ҪŪ������׼
	//auto icp_coarse = open3d::pipelines::registration::RegistrationICP(*pPcds[i], *pPcds[j],
	//	max_correspondence_distance_coarse, Eigen::Matrix4d::Identity(), open3d::pipelines::registration::TransformationEstimationPointToPlane());

	//��ϸICP��׼
	auto icpResult = open3d::pipelines::registration::RegistrationICP(*pPcds[i], *pPcds[j],
		max_correspondence_distance_fine, Eigen::Matrix4d::Identity(), open3d::pipelines::registration::TransformationEstimationPointToPlane());

	Eigen::Matrix4d transformation_icp = icpResult.transformation_;
	Eigen::Matrix6d information_icp = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(*pPcds[i], *pPcds[j], max_correspondence_distance_fine, icpResult.transformation_);

	odometry = transformation_icp * odometry;
	if (i < 3) poseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odometry.inverse()));
	poseGraph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(i, j, transformation_icp, information_icp, false));
	end = clock();
	if (isVerbose) std::cout << "	����ƴ�ӣ�" << frameName[i] << "��" << frameName[j] << "����ʱ��" << end - start << "����" << std::endl;
	if (isVerbose) std::cout << "		ICP��׼Fitness��" << icpResult.fitness_ << std::endl;
	if (isVerbose) std::cout << "		ICP��׼RMSE��" << icpResult.inlier_rmse_ << std::endl;
	return odometry;

}

void my::Alignment::getMatrice(std::string short1Path, std::string long2Path, std::string short3Path, std::string Long4Path)
{

	//step1. ��ȡ�ı��ļ�����ʽΪXYZ	
	if(isVerbose) 	std::cout << "��ȡ�̱�1��������..." << std::endl;
	readPLYData(short1Path);

	if (isVerbose) 	std::cout << "��ȡ����2��������..." << std::endl;
	readPLYData(long2Path);

	if (isVerbose) 	std::cout << "��ȡ�̱�3��������..." << std::endl;
	readPLYData(short3Path);

	if (isVerbose) 	std::cout << "��ȡ����4��������..." << std::endl;
	readPLYData(Long4Path);

	nPcds = pPcds.size();

	if (isVisualize)
	{
		pPcds[0]->PaintUniformColor(Eigen::Vector3d(1, 0.8, 0));
		pPcds[1]->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
		pPcds[2]->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
		pPcds[3]->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
	}

	//step2. ���ճ�ʼת�����󣬱任�߿����
	start = clock();
	time_t startAll = clock();
    // #pragma omp parallel
	for (size_t i = 0; i < nPcds; i++) {
		pPcds[i]->Transform(init_transformation[i]);
	}
	end = clock();
	if (isVerbose) 	std::cout << std::endl << "..................." << std::endl;
	if (isVerbose) 	std::cout << "�����߿���Ƴ�ʼ����ת����ʱ: " << end - start << "����" << std::endl;

	//step3. ��ȡ�ص�����,ͬʱ�²�����ȥ���쳣����޹ص�
	if (isVerbose) 	std::cout << std::endl << "..................." << std::endl;
	loadOverlap();

	if (isVisualize)
	{
		open3d::visualization::DrawGeometries({ pPcds[0] });
		open3d::visualization::DrawGeometries({ pPcds[1] });
		open3d::visualization::DrawGeometries({ pPcds[2] });
		open3d::visualization::DrawGeometries({ pPcds[3] });
	}	

	//step4. ����open3d����׼
	if (isVerbose) 	std::cout<<std::endl<< "..................." << std::endl;
	if (isVerbose) 	std::cout << "�߿����ƴ�Ӽ���..." << std::endl;
	fullRegistration();

	//step5. �洢��׼����
	for (int i = 0; i < nPcds; i++) 
	{
		Eigen::Matrix4d trans = poseGraph.nodes_[i].pose_;
		trans = trans * init_transformation[i];  //���Գ�ʼ����
		//std::cout <<std::endl<< trans << std::endl<<std::endl;
		finalTransformation.push_back(trans);


		pcdsOriginal[i].Transform(trans);
		*pPcds[i] = pcdsOriginal[i];

	}

	if (isVisualize) open3d::visualization::DrawGeometries({ pPcds[0],pPcds[1],pPcds[2],pPcds[3] });
	time_t endAll = clock();
	if (isVerbose) 	std::cout << std::endl << "..................." << std::endl;
	if (isVerbose) 	std::cout << "ƴ���ܺ�ʱ" << endAll-startAll<<"����"<<std::endl;
	return;
}

void my::Alignment::setVoxelSize(double size)
{
	if (size < 0.05 && size>0.10)
		size = 0.08;
	voxel_size = size;
	eps = 0.3;
	min_pt = int(130 - 1000 * size);
	max_correspondence_distance_coarse = size * 15;
	max_correspondence_distance_fine = size * 1.5;
	if (isVerbose) 	std::cout << "voxel_size:  " << voxel_size << std::endl << "min_pt:  " << min_pt << std::endl;
}

void my::Alignment::readPLYData(std::string filePath)
{	
	start = clock();
	std::shared_ptr<open3d::geometry::PointCloud> pPcdTemp = open3d::io::CreatePointCloudFromFile(filePath);
	end = clock();
	if (isVerbose) 	std::cout << "	ԭʼ���ƴ�С: " << pPcdTemp->points_.size() << std::endl;
	if (isVerbose) 	std::cout << "	���ݶ�ȡ��ʱ: " << end - start << "����" << std::endl;
	pPcds.push_back(pPcdTemp);
	pcdsOriginal.push_back(*pPcdTemp);
}

void my::Alignment::preprocessData(std::shared_ptr<open3d::geometry::PointCloud> &pPcd, bool isDownsampling, bool isRemovingOutLiner, bool isRemovingUnrelated)
{
	if(isDownsampling)
	{		
		if (isVerbose) 	std::cout << "	���ƴ�С: " << pPcd->points_.size() << std::endl;
		start = clock();
		pPcd = pPcd->VoxelDownSample(voxel_size);
		end = clock();
		if (isVerbose) 	std::cout << "	��������ʱ: " << (end - start) << "����" << std::endl;
		if (isVerbose) 	std::cout << "	����������ƴ�С " << pPcd->points_.size() << std::endl;
	}
	if (isRemovingOutLiner)
	{		
		std::vector<size_t> idx;
		start = clock();
		std::tie(pPcd, idx) = pPcd->RemoveStatisticalOutliers(40, 1.0, false);
		end = clock();
		if (isVerbose) 	std::cout << "	ȥ���쳣���ʱ: " << (end - start) << "����" << std::endl;
		if (isVerbose) 	std::cout << "	ȥ���쳣�����ƴ�С: " << pPcd->points_.size() << std::endl;
	}
	if (isRemovingUnrelated)
	{		
		start = clock();
		std::vector<int> labels = pPcd->ClusterDBSCAN(eps, min_pt, false);

		int max_label = *max_element(labels.begin(), labels.end());
		if (isVerbose) 	std::cout << "	���������Ϊ��" << max_label << std::endl;
		std::vector<open3d::geometry::PointCloud> clusters(max_label + 1);

		for (size_t i = 0; i < labels.size(); ++i) {
			if (labels[i] >= 0)
				clusters[labels[i]].points_.push_back(pPcd->points_[i]);
		}

		int max_idx = -1;
		int max_num = 0;
		for (int i = 0; i < clusters.size(); i++) {
			if (clusters[i].points_.size() > max_num) {
				max_num = clusters[i].points_.size();
				max_idx = i;
			}
		}

		end = clock();
		if (isVerbose) 	std::cout << "	�ָ��ʱ: " << (end - start) << " ����" << std::endl;
		if (isVerbose) 	std::cout << "	�ָ����ƴ�С��" << max_num << std::endl;
		*pPcd= clusters[max_idx];
	}
}
