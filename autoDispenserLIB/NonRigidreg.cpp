#pragma once
#include "pch.h"
#include "NonRigidreg.h"

typedef Eigen::Triplet<Scalar> Triplet;

NonRigidreg::NonRigidreg() {
};

NonRigidreg::~NonRigidreg()
{
}

void NonRigidreg::Initialize()
{
    ldlt_ = new Eigen::SimplicialCholesky<SparseMatrix>;
    nonrigid_init();
    // Initialize BFGS paras
    iter_ = 0;
    col_idx_ = 0;

    weight_d_.setOnes();  //n*n
    weight_s_.setOnes();
    Scalar sample_radius;

    Timer timer;
    Timer::EventID time1, time2;
    time1 = timer.get_time();
    // 是否直接读取文件
    if (pars_.read_vertexgraph)
        src_sample_nodes.ReadVertexGraph(*src_mesh_, pars_.out_each_step_info + "init_");
    else {
        sample_radius = src_sample_nodes.sampleAndconstuct(*src_mesh_, pars_.uni_sample_radio, svr::nodeSampler::X_AXIS);
        src_sample_nodes.print_nodes(*src_mesh_, pars_.out_each_step_info + "init_");  //保存文件
    }
   
    time2 = timer.get_time();

    // DEBUG: output node graph
    if (pars_.print_each_step_info)
    {
        std::string out_node = pars_.out_each_step_info + "init_";
        src_sample_nodes.print_nodes(*src_mesh_, out_node);//init sample nodes
    }

    num_sample_nodes = src_sample_nodes.nodeSize();  //r
    std::cout << num_sample_nodes << std::endl;

    pars_.num_sample_nodes = num_sample_nodes;

    // 初始化L_BFGS算法，通过两轮循环计算梯度方向
    all_s_.resize(4 * num_sample_nodes * 3, pars_.lbfgs_m); all_s_.setZero();  //使用最近m+1次迭代的X, s = X_(i+1)-X_i, 共有m个s, X是所有节点仿射变换的集合
    all_t_.resize(4 * num_sample_nodes * 3, pars_.lbfgs_m); all_t_.setZero();  //t=G(X_i+1)-G(X_i)  ,m个4r*3

    Smat_X_.resize(4 * num_sample_nodes, 3); Smat_X_.setZero();  //表示r个节点的仿射变换集合，4r*3
    Weight_PV_.resize(n_src_vertex_, 4 * num_sample_nodes);  // 计算F_ij, n*4r
    Smat_P_.resize(n_src_vertex_, 3);   //计算P n*3

    // G(E_rot) = 2(JX-Z)
    Smat_R_.resize(3 * num_sample_nodes, 3); Smat_R_.setZero();  // 3r*3 表示在旋转矩阵群上的投影矩阵Proj(R)
    Smat_L_.resize(4 * num_sample_nodes, 4 * num_sample_nodes);  //4r*4r   J 对角线为{1,1,1,0,1,1,1,0....}
    Smat_J_.resize(4 * num_sample_nodes, 3 * num_sample_nodes);  //4r*3r   Smat_J_*Smat_R_= Z 4r*3表示投影点坐标，用于计算投影偏差

    //Eigen中使用 Eigen::Triplet<Scalar>来记录一个非零元素的行、列、值，填充一个稀疏矩阵，
    //只需要将所有表示非零元素的Triplet放在一个 std::vector中即可传入即可
    std::vector<Triplet> coeffv(4 * num_sample_nodes);
    std::vector<Triplet> coeffL(3 * num_sample_nodes);
    std::vector<Triplet> coeffJ(3 * num_sample_nodes);
    // 将旋转矩阵全部初始化单位矩阵
    for (int i = 0; i < num_sample_nodes; i++)
    {
        // Smat_X_
        Smat_X_.block(4 * i, 0, 3, 3) = MatrixXX::Identity(3, 3);

        // Smat_R_
        Smat_R_.block(3 * i, 0, 3, 3) = MatrixXX::Identity(3, 3);

        // Smat_L_
        coeffL[3 * i] = Triplet(4 * i, 4 * i, 1.0);  //一个非零元素的行、列、值
        coeffL[3 * i + 1] = Triplet(4 * i + 1, 4 * i + 1, 1.0);
        coeffL[3 * i + 2] = Triplet(4 * i + 2, 4 * i + 2, 1.0);

        // Smat_J_
        coeffJ[3 * i] = Triplet(4 * i, 3 * i, 1.0);
        coeffJ[3 * i + 1] = Triplet(4 * i + 1, 3 * i + 1, 1.0);
        coeffJ[3 * i + 2] = Triplet(4 * i + 2, 3 * i + 2, 1.0);

    }
    Smat_L_.setFromTriplets(coeffL.begin(), coeffL.end());
    Smat_J_.setFromTriplets(coeffJ.begin(), coeffJ.end());
    direction_.resize(4 * num_sample_nodes, 3);  //所有节点仿射变换 X的梯度方向　4r*3

    // update coefficient matrices
    src_sample_nodes.initWeight(Weight_PV_, Smat_P_, Smat_B_, Smat_D_, Sweight_s_);
    if (pars_.use_landmark && pars_.landmark_src.size() > 0)
    {
        size_t n_landmarks = pars_.landmark_src.size();
        Sub_PV_.resize(n_landmarks, 4 * num_sample_nodes);
        Sub_UP_.resize(n_landmarks, 3);
        if (pars_.landmark_tar.size() != n_landmarks)
        {
            std::cout << "Error: The source and target points do not match!" << std::endl;
            exit(1);
        }
        for (size_t i = 0; i < n_landmarks; i++)
        {
            size_t src_idx = pars_.landmark_src[i];
            size_t tar_idx = pars_.landmark_tar[i];
            VectorX row_val = Weight_PV_.row(src_idx);
            for (int j = 0; j < 4 * num_sample_nodes; j++) {
                if (fabs(row_val[j]) > 1e-12) {
                    Sub_PV_.insert(i, j) = row_val[j];
                }
            }
            Sub_UP_.row(i) = tar_points_.col(tar_idx).transpose() - Smat_P_.row(src_idx);
        }
        pars_.gamma = pars_.gamma * n_landmarks / n_src_vertex_;
    }

    pars_.alpha = pars_.alpha * Smat_B_.rows() / n_src_vertex_; // alpha = k_alpha * (n/n_edge)
    pars_.beta = pars_.beta * num_sample_nodes / n_src_vertex_; // beta = k_beta * (n/r)
}

Scalar NonRigidreg::DoNonRigid()
{
    Scalar data_err, smooth_err, orth_err;

    // 记录前后的source点坐标
    MatrixXX curV = MatrixXX::Zero(n_src_vertex_, 3);
    MatrixXX prevV = MatrixXX::Zero(n_src_vertex_, 3);

    SparseMatrix Weight_PV0 = Weight_PV_;  // F
    SparseMatrix Smat_B0 = Smat_B_;
    MatrixXX	 Smat_D0 = Smat_D_;

    // welsch_sweight
    VectorX welsch_weight_s = VectorX::Ones(Sweight_s_.rows(), 1);
    bool run_once = true;

    Timer time;
    Timer::EventID begin_time, run_time;
    pars_.each_energys.clear();
    pars_.each_gt_mean_errs.clear();
    pars_.each_gt_max_errs.clear();
    pars_.each_times.clear();
    pars_.each_iters.clear();
    pars_.each_term_energy.clear();

    // print initial run results
    pars_.each_energys.push_back(0.0);
    pars_.each_gt_max_errs.push_back(pars_.init_gt_max_errs);
    pars_.each_gt_mean_errs.push_back(pars_.init_gt_mean_errs);
    pars_.each_iters.push_back(0);
    pars_.each_times.push_back(pars_.non_rigid_init_time);
    pars_.each_term_energy.push_back(Vector3(0, 0, 0));

    // Data term parameters
    Scalar nu1 = pars_.Data_initk * pars_.Data_nu;   // v_a^max=10d
    Scalar average_len = CalcEdgelength(src_mesh_, 1);
    Scalar end_nu1 = pars_.Data_endk * average_len;  // v_a^min=0.5L
    Scalar nu2 = pars_.Smooth_nu * average_len;    // v_r^max=40L

    // Smooth term parameters
    ori_alpha = pars_.alpha;
    ori_beta = pars_.beta;
    //相当于对所有项同乘2 * v_a^2
    pars_.alpha = ori_alpha * nu1 * nu1 / (nu2 * nu2);
    pars_.beta = ori_beta * 2.0 * nu1 * nu1;

    Scalar gt_err;

    bool dynamic_stop = false;
    int accumulate_iter = 0;

    begin_time = time.get_time();
    while (!dynamic_stop)
    {
        for (int out_iter = 0; out_iter < pars_.max_outer_iters; out_iter++)
        {
            // according correspondence_pairs to update mat_U0_;
            mat_U0_.setZero();  //n*3, 每行保存对应点对中的target点
            corres_pair_ids_.setZero();  //n 记录该source顶点是否有最近邻点
            //correspondence_pairs_为对应点对，数量远小于n
#pragma omp parallel for
            for (int i = 0; i < correspondence_pairs_.size(); i++)
            {
                mat_U0_.col(correspondence_pairs_[i].src_idx) = correspondence_pairs_[i].position;
                corres_pair_ids_[correspondence_pairs_[i].src_idx] = 1;
            }

            // update weight
            if (pars_.data_use_welsch)
            {
                weight_d_ = (Weight_PV0 * Smat_X_ + Smat_P_ - mat_U0_.transpose()).rowwise().norm(); //n*3的二范数得到n向量, ||FX+P-U||2
                welsch_weight(weight_d_, nu1);
            }
            else
                weight_d_.setOnes();

            if (pars_.smooth_use_welsch)
            {
                welsch_weight_s = ((Smat_B0 * Smat_X_ - Smat_D0)).rowwise().norm();
                welsch_weight(welsch_weight_s, nu2);
            }
            else
                welsch_weight_s.setOnes();

            // int welsch_iter;
            int total_inner_iters = 0;
            MatrixXX old_X = Smat_X_;

            // update V,U and D
            weight_d_ = weight_d_.cwiseSqrt().cwiseProduct(corres_pair_ids_);
            Weight_PV_ = weight_d_.asDiagonal() * Weight_PV0;
            welsch_weight_s = welsch_weight_s.cwiseSqrt().cwiseProduct(Sweight_s_);
            Smat_B_ = welsch_weight_s.asDiagonal() * Smat_B0;

            update_R();

            Smat_D_ = welsch_weight_s.asDiagonal() * Smat_D0;
            Smat_UP_ = weight_d_.asDiagonal() * (mat_U0_.transpose() - Smat_P_);

            // construct matrix A0 and pre-decompose
            // 4r*4r H0=2(F^T * W_d^2 * F + alpha * B^T * W_s^2 * B + beta * J)
            mat_A0_ = Weight_PV_.transpose() * Weight_PV_
                + pars_.alpha * Smat_B_.transpose() * Smat_B_
                + pars_.beta * Smat_L_;
            // auxiliary variable 4r * 3
            // 用于与H0一起求梯度G(X)
            mat_VU_ = (Weight_PV_).transpose() * Smat_UP_
                + pars_.alpha * (Smat_B_).transpose() * Smat_D_;

            if (pars_.use_landmark)
            {
                mat_A0_ += pars_.gamma * Sub_PV_.transpose() * Sub_PV_;
                mat_VU_ += pars_.gamma * Sub_PV_.transpose() * Sub_UP_;
            }

            if (run_once)
            {
                ldlt_->analyzePattern(mat_A0_);   //函数analyzePattern()的目的是对矩阵的非零元素重新排序，用以减轻分解步的病态性
                run_once = false;
            }
            // 对H0分解用于后面计算
            ldlt_->factorize(mat_A0_); //函数factorize()中计算矩阵的分解因子.当矩阵变化时，都应该调用这步.

            if (!pars_.use_lbfgs)
            {
                MatrixXX b = pars_.beta * Smat_J_ * Smat_R_ + mat_VU_;
#pragma omp parallel for
                for (int col_id = 0; col_id < 3; col_id++)
                {
                    Smat_X_.col(col_id) = ldlt_->solve(b.col(col_id));
                }
                total_inner_iters += 1;
            }
            else
            {
                total_inner_iters += QNSolver(data_err, smooth_err, orth_err);
            }

            MatrixXX target = Weight_PV0 * Smat_X_ + Smat_P_;
            gt_err = SetMeshPoints(src_mesh_, target, curV);

            run_time = time.get_time();
            pars_.each_gt_mean_errs.push_back(gt_err);
            pars_.each_gt_max_errs.push_back(0);

            pars_.each_energys.push_back(0.0);
            double eps_time = time.elapsed_time(begin_time, run_time);
            pars_.each_times.push_back(eps_time);
            pars_.each_iters.push_back(total_inner_iters);
            pars_.each_term_energy.push_back(Vector3(data_err, smooth_err, orth_err));

            if (pars_.print_each_step_info)
            {
                std::string out_obj = pars_.out_each_step_info + "iter_obj_" + std::to_string(accumulate_iter) + ".ply";

                OpenMesh::IO::write_mesh(*src_mesh_, out_obj.c_str());
                std::cout << "iter = " << out_iter << " time = " << eps_time
                    << "  inner iter = " << total_inner_iters << std::endl;
            }

            // Find clost points
            FindClosestPoints(correspondence_pairs_);
            accumulate_iter++;
            SimplePruning(correspondence_pairs_, pars_.use_distance_reject, pars_.use_normal_reject);

            // stop condition should be revised
            if ((curV - prevV).rowwise().norm().maxCoeff() < pars_.stop)
            {
                break;
            }
            prevV = curV;
        }
        // 如果v_a到了最小值
        if (fabs(nu1 - end_nu1) < 1e-8 || !pars_.use_Dynamic_nu || !pars_.data_use_welsch)
            dynamic_stop = true;
        // v_a, v_r 减半
        nu1 = (0.5 * nu1 > end_nu1) ? 0.5 * nu1 : end_nu1;
        nu2 *= 0.5;
        pars_.alpha = ori_alpha * nu1 * nu1 / (nu2 * nu2);
        pars_.beta = ori_beta * 2 * nu1 * nu1;
    }
    return 0;
}

Scalar NonRigidreg::sample_energy(Scalar& data_err, Scalar& smooth_err, Scalar& orth_err)
{
    // E_align = ||W_d(FX+P-U)||2
    data_err = (Weight_PV_ * Smat_X_ - Smat_UP_).squaredNorm();
    // E_reg = ||W_s(BX-Y)||2
    smooth_err = ((Smat_B_ * Smat_X_ - Smat_D_)).squaredNorm();
    VectorX orth_errs(num_sample_nodes);
#pragma omp parallel for
    for (int i = 0; i < num_sample_nodes; i++)
    {
        Eigen::JacobiSVD<MatrixXX> svd(Smat_X_.block(4 * i, 0, 3, 3), Eigen::ComputeThinU | Eigen::ComputeThinV);

        if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
            Vector3 S = Vector3::Ones(); S(2) = -1.0;
            Smat_R_.block(3 * i, 0, 3, 3) = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
        }
        else {
            Smat_R_.block(3 * i, 0, 3, 3) = svd.matrixU() * svd.matrixV().transpose();
        }
        orth_errs[i] = (Smat_X_.block(4 * i, 0, 3, 3) - Smat_R_.block(3 * i, 0, 3, 3)).squaredNorm();
    }
    // E_rot = sum(||A-Proj(A)||2)
    orth_err = orth_errs.sum();
    Scalar total_err = data_err + pars_.alpha * smooth_err + pars_.beta * orth_err;
    if (pars_.use_landmark)
        total_err += pars_.gamma * (Sub_PV_ * Smat_X_ - Sub_UP_).squaredNorm();
    return total_err;
}

void NonRigidreg::update_R()
{
#pragma omp parallel for
    for (int i = 0; i < num_sample_nodes; i++)
    {
        Eigen::JacobiSVD<MatrixXX> svd(Smat_X_.block(4 * i, 0, 3, 3), Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
            Vector3 S = Vector3::Ones(); S(2) = -1.0;
            Smat_R_.block(3 * i, 0, 3, 3) = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
        }
        else {
            Smat_R_.block(3 * i, 0, 3, 3) = svd.matrixU() * svd.matrixV().transpose();
        }
    }
}

void NonRigidreg::sample_gradient()
{
    grad_X_ = 2 * (mat_A0_ * Smat_X_ - mat_VU_ - pars_.beta * Smat_J_ * Smat_R_);
}

// col vector
void NonRigidreg::LBFGS(int iter, MatrixXX& dir) const
{
    VectorX rho(pars_.lbfgs_m);
    VectorX kersi(pars_.lbfgs_m);
    MatrixXX q(4 * num_sample_nodes, 3);
    MatrixXX temp(4 * num_sample_nodes, 3);
    int k = iter;
    q.setZero();
    dir = q;
    q = -grad_X_;
    int m_k = std::min(k, pars_.lbfgs_m);
    // 使用最近的m+1进行计算
    for (int i = k - 1; i > k - m_k - 1; i--)
    {
        int col = (pars_.lbfgs_m + col_idx_ - (k - 1 - i)) % pars_.lbfgs_m;
        // rho = Tr(t_i^T * s_i)
        rho(k - 1 - i) = all_t_.col(col).transpose().dot(all_s_.col(col));
        // kersi = Tr(s_i^T * q) / rho
        Scalar lbfgs_err_scalar = Eigen::Map<VectorX>(q.data(), q.size()).dot(all_s_.col(col));
        kersi(k - 1 - i) = lbfgs_err_scalar / rho(k - 1 - i);
        // q = q - kersi * t
        Eigen::Map<VectorX>(q.data(), q.size()) -= kersi(k - 1 - i) * all_t_.col(col);
    }
    // d = H0^(-1) * q
#pragma omp parallel for
    for (int cid = 0; cid < 3; cid++)
    {
        dir.col(cid) = ldlt_->solve(q.col(cid));
    }

    for (int i = k - m_k; i < k; i++)
    {
        int col = (pars_.lbfgs_m + col_idx_ - (k - 1 - i)) % pars_.lbfgs_m;
        Scalar lbfgs_err_scalar = all_t_.col(col).dot(Eigen::Map<VectorX>(dir.data(), dir.size()));
        Scalar eta = kersi(k - 1 - i) - lbfgs_err_scalar / rho(k - 1 - i);
        // d = d + s(kersi - Tr(t*d) / rho)
        Eigen::Map<VectorX>(dir.data(), dir.size()) += all_s_.col(col) * eta;
    }

    rho.resize(0);
    kersi.resize(0);
    q.resize(0, 0);
    temp.resize(0, 0);
    return;
}


int NonRigidreg::QNSolver(Scalar& data_err, Scalar& smooth_err, Scalar& orth_err)
{
    MatrixXX prev_X;
    int count_linesearch = 0;

    int iter;
    for (iter = 0; iter <= pars_.max_inner_iters; iter++)
    {
        //EvalGradient();
        sample_gradient();

        // update decent direction
        if (iter == 0)
        {
            MatrixXX temp = -grad_X_;
#pragma omp parallel for
            for (int cid = 0; cid < 3; cid++)
            {
                direction_.col(cid) = ldlt_->solve(temp.col(cid));
            }

            col_idx_ = 0;
            all_s_.col(col_idx_) = -Eigen::Map<VectorX>(Smat_X_.data(), 4 * num_sample_nodes * 3);
            all_t_.col(col_idx_) = -Eigen::Map<VectorX>(grad_X_.data(), 4 * num_sample_nodes * 3);
        }
        else
        {
            all_s_.col(col_idx_) += Eigen::Map<VectorX>(Smat_X_.data(), 4 * num_sample_nodes * 3);
            all_t_.col(col_idx_) += Eigen::Map<VectorX>(grad_X_.data(), 4 * num_sample_nodes * 3);

            // Get descent direction
            LBFGS(iter, direction_);
            col_idx_ = (col_idx_ + 1) % pars_.lbfgs_m;
            all_s_.col(col_idx_) = -Eigen::Map<VectorX>(Smat_X_.data(), 4 * num_sample_nodes * 3);
            all_t_.col(col_idx_) = -Eigen::Map<VectorX>(grad_X_.data(), 4 * num_sample_nodes * 3);
        }
        // 使用线性搜索方法找到一个步长alpha，使得足够的梯度下降 
        Scalar alpha = 2.0;
        prev_X = Smat_X_;
        Scalar new_err = sample_energy(data_err, smooth_err, orth_err);  
        Scalar prev_err = new_err;
        Scalar gamma = 0.3;
        // 使其满足 E_(j+1) <= E_j + alpha*gamma*Tr(G_j*d_j)
        Scalar x = (grad_X_.transpose() * direction_).trace();
        do
        {
            alpha /= 2;
            Smat_X_ = prev_X + alpha * direction_;
            new_err = sample_energy(data_err, smooth_err, orth_err);
            count_linesearch++;
        } while (new_err > prev_err + gamma * alpha * x);
        // 当满足条件时，退出L-BFGS solver
        if (fabs(new_err - prev_err) < pars_.stop)
        {
            break;
        }
        iter_++;
    }
    return iter;
}

Scalar NonRigidreg::welsch_error(Scalar nu1, Scalar nu2)
{
    VectorX w_data = (Weight_PV_ * Smat_X_ - Smat_UP_).rowwise().norm();
    Scalar data_err;
    if (pars_.data_use_welsch)
        data_err = welsch_energy(w_data, nu1);
    else
        data_err = w_data.squaredNorm();
    VectorX s_data = ((Smat_B_ * Smat_X_ - Smat_D_)).rowwise().norm();
    Scalar smooth_err;
    if (pars_.smooth_use_welsch)
        smooth_err = welsch_energy(s_data, nu2);
    else
        smooth_err = s_data.squaredNorm();

    VectorX orth_errs(num_sample_nodes);
#pragma omp parallel for
    for (int i = 0; i < num_sample_nodes; i++)
    {
        orth_errs[i] = (Smat_X_.block(4 * i, 0, 3, 3) - Smat_R_.block(3 * i, 0, 3, 3)).squaredNorm();
    }
    return data_err + pars_.alpha * smooth_err + pars_.beta * orth_errs.sum();
}

Scalar NonRigidreg::welsch_energy(VectorX& r, Scalar p) {
    VectorX energy(r.rows());
#pragma omp parallel for
    for (int i = 0; i < r.rows(); ++i) {
        energy[i] = 1.0 - std::exp(-r(i) * r(i) / (2 * p * p));
    }
    return energy.sum();
}

void NonRigidreg::welsch_weight(VectorX& r, Scalar p) {
#pragma omp parallel for
    for (int i = 0; i < r.rows(); ++i) {
        r(i) = std::exp(-r(i) * r(i) / (2 * p * p));
    }
}

Scalar NonRigidreg::SetMeshPoints(Mesh* mesh, const MatrixXX& target, MatrixXX& cur_v)
{
    VectorX gt_errs(n_src_vertex_);
#pragma omp parallel for
    for (int i = 0; i < n_src_vertex_; i++)
    {
        MatrixXX tar_p = target.block(i, 0, 1, 3);
        Vec3 p(tar_p(0, 0), tar_p(0, 1), tar_p(0, 2));
        mesh->set_point(mesh->vertex_handle(i), p);
        cur_v.row(i) = tar_p;
        if (pars_.calc_gt_err)
            gt_errs[i] = (tar_p.transpose() - tar_points_.col(i)).squaredNorm();
    }
    if (pars_.calc_gt_err)
        return gt_errs.sum() / n_src_vertex_;
    else
        return -1.0;
}
