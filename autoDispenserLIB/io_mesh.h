#pragma once
#include <iostream>
#include <fstream>
#include "tools/Types.h"

enum { TRIANGLE = 0, QUAD, N_MESH_MODES };
int checkMeshMode(Mesh& mesh);

void printBasicMeshInfo(Mesh& mesh);

bool read_data(const std::string filename, Mesh& mesh, bool to_triangle = true);

bool write_data(const char* filename, Mesh& mesh, Scalar scale);


std::string num2str(int num, const int size, bool is_add0);
