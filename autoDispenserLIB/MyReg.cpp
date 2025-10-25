#include "pch.h"
#include "MyReg.h"

my::MyReg::MyReg()
{
}

my::MyReg::~MyReg()
{
}

void my::MyReg::SetParas(Scalar radio, bool save, std::string out_each_step_info)
{
    // Setting paras
    paras.alpha = 100.0;
    paras.beta = 100.0;
    paras.gamma = 1e8;
    paras.uni_sample_radio = radio;  // 5.0

    paras.use_distance_reject = true;
    paras.distance_threshold = 0.05;  // 0.05;
    paras.use_normal_reject = true;  //false;
    paras.normal_threshold = M_PI / 3;
    paras.use_Dynamic_nu = true;

    //print
    paras.read_vertexgraph = save;
    paras.out_each_step_info = out_each_step_info;

}

void my::MyReg::ReadMesh(std::string src_file, std::string tar_file)
{
	read_data(src_file, src_mesh);
	read_data(tar_file, tar_mesh);

	if (src_mesh.n_vertices() == 0 || tar_mesh.n_vertices() == 0)
		exit(0);
	if (src_mesh.n_vertices() != tar_mesh.n_vertices()) 
		paras.calc_gt_err = false;

	// mesh 点坐标尺度归一化
	scale = mesh_scaling(src_mesh, tar_mesh);
    std::cout << "scale=   " << scale << std::endl;
}

void my::MyReg::DoRegistration()
{
    NonRigidreg* reg;
    reg = new NonRigidreg;

    std::cout << "\nrigid registration to initial..." << std::endl;
    reg->rigid_init(src_mesh, tar_mesh, paras);
    reg->DoRigid();
    std::cout << "rgid registration... " << std::endl;
    // non-rigid initialize
    std::cout << "non-rigid registration to initial..." << std::endl;
    reg->Initialize();
    std::cout << "non-rigid registration... " << std::endl;
    reg->DoNonRigid();

    delete reg;
}

void my::MyReg::SaveMesh(std::string out_file)
{
    write_data(out_file.c_str(), src_mesh, scale);
    std::cout << "write result to " << out_file << std::endl;
}