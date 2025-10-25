#pragma once
#include <string>
#include <time.h>
#include "io_mesh.h"
#include "tools/OmpHelper.h"
#include "NonRigidreg.h"
#include <fstream>
#include <iostream>

namespace my
{
    class MyReg
    {
    public:
        MyReg();
        ~MyReg();
        void SetParas(Scalar radio, bool print, std::string out_each_step_info);
        void ReadMesh(std::string src_file, std::string tar_file);
        void DoRegistration();
        void SaveMesh(std::string out_file);

    private:
        Mesh src_mesh;
        Mesh tar_mesh;
        double scale;

        RegParas paras;

    };
}


