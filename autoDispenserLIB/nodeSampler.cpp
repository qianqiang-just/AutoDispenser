//#pragma once
#include "pch.h"
#include "nodeSampler.h"
#include "tools.h"

#include "geodesic/geodesic_algorithm_exact.h"

namespace svr
{
    //	Define helper functions
    static auto square = [](const Scalar argu) { return argu * argu; };
    static auto cube = [](const Scalar argu) { return argu * argu * argu; };
    static auto max = [](const Scalar lhs, const Scalar rhs) { return lhs > rhs ? lhs : rhs; };

    //------------------------------------------------------------------------
    //	Node Sampling based on geodesic distance metric
    //
    //	Note that this member function samples nodes along some axis.
    //	Each node is not covered by any other node. And distance between each
    //	pair of nodes is at least sampling radius.
    //------------------------------------------------------------------------


    // Local geodesic calculation
    Scalar nodeSampler::sampleAndconstuct(Mesh &mesh, Scalar sampleRadiusRatio, sampleAxis axis)
    {
        //	Save numbers of vertex and edge
        m_meshVertexNum = mesh.n_vertices();
        m_meshEdgeNum = mesh.n_edges();
        m_mesh = &mesh;

        //	Calculate average edge length of bound mesh
        for (size_t i = 0; i < m_meshEdgeNum; ++i)
        {
            OpenMesh::EdgeHandle eh = mesh.edge_handle(i);
            Scalar edgeLen = mesh.calc_edge_length(eh);
            m_averageEdgeLen += edgeLen;
        }
        m_averageEdgeLen /= m_meshEdgeNum;

        //	Sampling radius is calculated as averageEdgeLen multiplied by sampleRadiusRatio
        // $R$ 默认设置为5L，其中L为source mesh的平均边的长度
        m_sampleRadius = sampleRadiusRatio * m_averageEdgeLen;
        std::cout << sampleRadiusRatio << std::endl;
        std::cout << m_sampleRadius << std::endl;

        //	Reorder mesh vertex along axis
        std::vector<size_t> vertexReorderedAlongAxis(m_meshVertexNum);
        size_t vertexIdx = 0;
        std::generate(vertexReorderedAlongAxis.begin(), vertexReorderedAlongAxis.end(), [&vertexIdx]() -> size_t { return vertexIdx++; });
        // 根据source points 对协方差矩阵最大特征值轴的投影进行排序
        time_t start, end;
        start = clock();
        std::sort(vertexReorderedAlongAxis.begin(), vertexReorderedAlongAxis.end(), [&mesh, axis](const size_t &lhs, const size_t &rhs) -> bool {
            size_t lhsIdx = lhs;
            size_t rhsIdx = rhs;
            OpenMesh::VertexHandle vhl = mesh.vertex_handle(lhsIdx);
            OpenMesh::VertexHandle vhr = mesh.vertex_handle(rhsIdx);
            Mesh::Point vl = mesh.point(vhl);
            Mesh::Point vr = mesh.point(vhr);
            return vl[axis] > vr[axis];
        });
        end = clock();
        std::cout << "排序所用时间为：" << end - start << "毫秒。" << std::endl;
        //	Sample nodes using radius of m_sampleRadius
        size_t firstVertexIdx = vertexReorderedAlongAxis[0];
        VertexNodeIdx.resize(m_meshVertexNum);  // 记录所有顶点对应的节点[0-(r-1)]共r个节点，不是节点默认为-1
        VertexNodeIdx.setConstant(-1);
        VertexNodeIdx[firstVertexIdx] = 0;
        size_t cur_node_idx = 0;   // 表示节点index

        m_vertexGraph.resize(m_meshVertexNum);   // 记录所有顶点受节点的权重影响
        VectorX weight_sum = VectorX::Zero(m_meshVertexNum);

        for (auto &vertexIdx : vertexReorderedAlongAxis)
        {
            if(VertexNodeIdx[vertexIdx] < 0 && m_vertexGraph.at(vertexIdx).empty())
            {
                m_nodeContainer.emplace_back(cur_node_idx, vertexIdx);  // 记录节点index所对应的顶点index
                VertexNodeIdx[vertexIdx] = cur_node_idx;

                std::vector<size_t> neighbor_verts;  // 该节点近邻的所有顶点
                geodesic::GeodesicAlgorithmExact geoalg(&mesh, vertexIdx, m_sampleRadius);
                geoalg.propagate(vertexIdx, neighbor_verts);
                for(size_t i = 0; i < neighbor_verts.size(); i++)
                {
                    int neighIdx = neighbor_verts[i];
                    Scalar geodist = mesh.data(mesh.vertex_handle(neighIdx)).geodesic_distance;
                    if(geodist < m_sampleRadius)  //如果测地距离 < R
                    {
                        Scalar weight = std::pow(1-std::pow(geodist/m_sampleRadius, 2), 3);   //计算该邻居顶点受当前节点的影响权重
                        m_vertexGraph.at(neighIdx).emplace(std::pair<int, Scalar>(cur_node_idx, weight));
                        weight_sum[neighIdx] += weight;   // 将所有节点对该顶点的权重进行求和，计算平均权重
                    }
                }
                cur_node_idx++;
            }
        }

        m_nodeGraph.resize(cur_node_idx); //记录每个节点的邻居节点
        for (auto &vertexIdx : vertexReorderedAlongAxis)
        {
            for(auto &node: m_vertexGraph[vertexIdx])  
            {
                //对所有影响该顶点的节点，当作邻居节点
                size_t nodeIdx = node.first;
                for(auto &neighNode: m_vertexGraph[vertexIdx])
                {
                    size_t neighNodeIdx = neighNode.first;
                    if(nodeIdx != neighNodeIdx)
                    {
                        m_nodeGraph.at(nodeIdx).emplace(std::pair<int, Scalar>(neighNodeIdx, 1.0));
                    }
                }
                m_vertexGraph.at(vertexIdx).at(nodeIdx) /= weight_sum[vertexIdx]; //计算平均权重
            }
        }
        return m_sampleRadius;
    }



    void nodeSampler::initWeight(Eigen::SparseMatrix<Scalar>& matPV, MatrixXX & matP,
        Eigen::SparseMatrix<Scalar>& matB, MatrixXX& matD, VectorX& smoothw)
    {
        std::vector<Eigen::Triplet<Scalar>> coeff;
        matP.setZero();
        Eigen::VectorXi nonzero_num = Eigen::VectorXi::Zero(m_mesh->n_vertices());
        // data coeff
        for (size_t vertexIdx = 0; vertexIdx < m_meshVertexNum; ++vertexIdx)
        {
            Mesh::Point vi = m_mesh->point(m_mesh->vertex_handle(vertexIdx));
            for (auto &eachNeighbor : m_vertexGraph[vertexIdx])
            {
                size_t nodeIdx = eachNeighbor.first;
                Scalar weight = m_vertexGraph.at(vertexIdx).at(nodeIdx);
                Mesh::Point pj = m_mesh->point(m_mesh->vertex_handle(getNodeVertexIdx(nodeIdx)));
                //计算F_ij, n*4r
                coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4, weight * (vi[0] - pj[0])));
                coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 1, weight * (vi[1] - pj[1])));
                coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 2, weight * (vi[2] - pj[2])));
                coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 3, weight * 1.0));
                // 计算P
                matP(vertexIdx, 0) += weight * pj[0];
                matP(vertexIdx, 1) += weight * pj[1];
                matP(vertexIdx, 2) += weight * pj[2];
            }
            nonzero_num[vertexIdx] = m_vertexGraph[vertexIdx].size();  //记录每个顶点受多少个节点影响
        }
        matPV.setFromTriplets(coeff.begin(), coeff.end()); //n*4r


        //trajectory
        //coeff.clear();
        //trajP.setZero();
        //for (size_t vertexIdx = 0; vertexIdx < traj_meshVertexNum; ++vertexIdx)
        //{
        //    //轨迹点坐标
        //    Mesh::Point vi = traj_mesh->point(traj_mesh->vertex_handle(vertexIdx));
        //    for (auto& eachNeighbor : traj_vertexGraph[vertexIdx])
        //    {
        //        size_t nodeIdx = eachNeighbor.first;
        //        Scalar weight = traj_vertexGraph.at(vertexIdx).at(nodeIdx);
        //        // 获取source中节点的点坐标
        //        Mesh::Point pj = m_mesh->point(m_mesh->vertex_handle(getNodeVertexIdx(nodeIdx)));
        //        //len*4r
        //        coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4, weight * (vi[0] - pj[0])));
        //        coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 1, weight * (vi[1] - pj[1])));
        //        coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 2, weight * (vi[2] - pj[2])));
        //        coeff.push_back(Eigen::Triplet<Scalar>(vertexIdx, nodeIdx * 4 + 3, weight * 1.0));
        //        // 计算P
        //        trajP(vertexIdx, 0) += weight * pj[0];
        //        trajP(vertexIdx, 1) += weight * pj[1];
        //        trajP(vertexIdx, 2) += weight * pj[2];
        //    }
        //}
        //mat_traj.setFromTriplets(coeff.begin(), coeff.end()); //len*4r

        // smooth coeff
        coeff.clear();
        int max_edge_num = nodeSize() * (nodeSize()-1);  // 2*n_edge，表示2倍的边数，边数最大为r*(r-1)
        matB.resize(max_edge_num, 4 * nodeSize());   // B 2n_edge*4r
        matD.resize(max_edge_num, 3);   //D 2n_edge*3
        smoothw.resize(max_edge_num);
        matD.setZero();
        int edge_id = 0;
        for (size_t nodeIdx = 0; nodeIdx < m_nodeContainer.size(); ++nodeIdx)
        {
            size_t vIdx0 = getNodeVertexIdx(nodeIdx);
            Mesh::VertexHandle vh0 = m_mesh->vertex_handle(vIdx0);
            Mesh::Point v0 = m_mesh->point(vh0);
            for (auto &eachNeighbor : m_nodeGraph[nodeIdx])
            {
                size_t neighborIdx = eachNeighbor.first;
                size_t vIdx1 = getNodeVertexIdx(neighborIdx);
                Mesh::Point v1 = m_mesh->point(m_mesh->vertex_handle(vIdx1));
                Mesh::Point dv = v0 - v1;
                int k = edge_id;

                //每一行有两个(v0 - v1, 1)和(0,0,0,-1)，表示这条边的头尾节点，并在尾节点记录下v0 - v1
                //所以总共会有2*n_edge行数据，每行为一个半边
                coeff.push_back(Eigen::Triplet<Scalar>(k, neighborIdx * 4, dv[0]));
                coeff.push_back(Eigen::Triplet<Scalar>(k, neighborIdx * 4 + 1, dv[1]));
                coeff.push_back(Eigen::Triplet<Scalar>(k, neighborIdx * 4 + 2, dv[2]));
                coeff.push_back(Eigen::Triplet<Scalar>(k, neighborIdx * 4 + 3, 1.0));
                coeff.push_back(Eigen::Triplet<Scalar>(k, nodeIdx * 4 + 3, -1.0));

                Scalar dist = dv.norm();
                if(dist > 0)
                {
                    smoothw[k] = 1.0/ dist;
                }
                else
                {
                    smoothw[k] = 0.0;
                    std::cout << "node repeat";
                    exit(1);
                }
                matD(k, 0) = dv[0];
                matD(k, 1) = dv[1];
                matD(k, 2) = dv[2];
                edge_id++;
            }
        }
        matB.setFromTriplets(coeff.begin(), coeff.end());
        matD.conservativeResize(edge_id, 3);
        matB.conservativeResize(edge_id, matPV.cols());
        smoothw.conservativeResize(edge_id);
        smoothw *= edge_id/(smoothw.sum());
    }


    void nodeSampler::print_nodes(Mesh & mesh, std::string file_path)
    {
        std::string namev = file_path + "nodes.obj";
        std::ofstream out1(namev);
        //std::cout << "print nodes to " << name << std::endl;
        for (size_t i = 0; i < m_nodeContainer.size(); i++)
        {
            int vexid = m_nodeContainer[i].second;
            out1 << "v " << mesh.point(mesh.vertex_handle(vexid))[0] << " " << mesh.point(mesh.vertex_handle(vexid))[1]
                << " " << mesh.point(mesh.vertex_handle(vexid))[2] << std::endl;
        }
        Eigen::VectorXi nonzero_num = Eigen::VectorXi::Zero(m_nodeContainer.size());
        for (size_t nodeIdx = 0; nodeIdx < m_nodeContainer.size(); ++nodeIdx)
        {
            for (auto &eachNeighbor : m_nodeGraph[nodeIdx])
            {
                size_t neighborIdx = eachNeighbor.first;
                out1 << "l " << nodeIdx+1 << " " << neighborIdx+1 << std::endl;
            }
            nonzero_num[nodeIdx] = m_nodeGraph[nodeIdx].size();
        }
        std::cout << "node neighbor min = " << nonzero_num.minCoeff() << " max = "
                  << nonzero_num.maxCoeff() << " average = " << nonzero_num.mean() << std::endl;
        out1.close();
        std::string namee = file_path + "edges.txt";
        std::ofstream out2(namee);
        for (size_t nodeIdx = 0; nodeIdx < m_nodeContainer.size(); ++nodeIdx)
        {
            size_t vIdx0 = getNodeVertexIdx(nodeIdx);
            for (auto &eachNeighbor : m_nodeGraph[nodeIdx])
            {
                size_t neighborIdx = eachNeighbor.first;
                size_t vIdx1 = getNodeVertexIdx(neighborIdx);
                out2 << nodeIdx << " " << neighborIdx << " " << vIdx0 << " " << vIdx1 << std::endl;
            }
        }
        out2.close();

        //保存m_vertexGraph
        std::string vertex_namee = file_path + "vertex.txt";
        std::ofstream out3(vertex_namee);
        for (size_t verIdx = 0; verIdx < m_meshVertexNum; ++verIdx)
        {
            for (auto& eachNeighbor : m_vertexGraph[verIdx])
            {
                size_t nodeIdx = eachNeighbor.first;
                Scalar weight = m_vertexGraph.at(verIdx).at(nodeIdx);

                out3 << verIdx << " " << nodeIdx << " " << weight << std::endl;
            }
        }
        out3.close();
    }
    void nodeSampler::ReadVertexGraph(Mesh& mesh, std::string path)
    {
        //	Save numbers of vertex and edge
        m_meshVertexNum = mesh.n_vertices();
        m_meshEdgeNum = mesh.n_edges();
        m_mesh = &mesh;

        std::ifstream file1(path + "vertex.txt");
        size_t verIdx, nodeIdx;
        Scalar weight;
        m_vertexGraph.resize(m_meshVertexNum);   // 记录所有顶点受节点的权重影响

        int node_num = 0;
        while (file1 >> verIdx >> nodeIdx >> weight) {
            if (nodeIdx > node_num)
                node_num = nodeIdx;
            m_vertexGraph.at(verIdx).emplace(std::pair<int, Scalar>(nodeIdx, weight));
        }
        file1.close();

        node_num++;
        std::ifstream file2(path + "edges.txt");
        m_nodeContainer.resize(node_num);
        m_nodeGraph.resize(node_num); //记录每个节点的邻居节点
        size_t neighborIdx, vIdx0, vIdx1;
        while (file2 >> nodeIdx >> neighborIdx >> vIdx0>>vIdx1) {
            m_nodeContainer.at(nodeIdx) = std::pair<int, int>(nodeIdx, vIdx0);
            m_nodeGraph.at(nodeIdx).emplace(std::pair<int, Scalar>(neighborIdx, 1.0));
        }
        file2.close();
       
    }
}
