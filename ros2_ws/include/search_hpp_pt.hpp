
/*******************************************
 * HPP with Probabilistic Terminals
 * Author: Yunpeng Lyu, Zhongqiang Ren
 * Ownership: RAP-lab@SJTU
 * All Rights Reserved. 
 *******************************************/


#ifndef RAPLAB_BASIC_SEARCH_HPP_PT_H_
#define RAPLAB_BASIC_SEARCH_HPP_PT_H_

#include "search_emoa.hpp"

namespace raplab{

#define DEBUG_HPPPT 0
#define IF_HEURISTIC 1
#define P_NORMALIZATION 0
#define DETECTION_FAILURE_ALLOWED 0
#define SUB_OPTIMAL 0
#define REWRITE 0 //result rewrite

// /**
//  * @brief
//  */
struct HPPPTResult {
  long id;
  CostVec cost;
  std::vector<long> path;
  long n_generated = 0;
  long n_expanded = 0;
  long n_PruneCheck = 0;
  double rt_initHeu = 0.0;
  double rt_search = 0.0;
  bool timeout = false;
  double num_nondom_labels_avg = -1;
  double num_nondom_labels_max = -1;
  std::string name;
  double time_limit = 60;
};
/**
 * @brief
 */
class AstarHPPPT : public EMOA
{
public:
  AstarHPPPT();
  virtual ~AstarHPPPT();

  // override the method in EMOA class if needed
  // virtual HPPPTResult GetResult() const ;//can't use
  virtual HPPPTResult GetHPPPTResult() const ;

  virtual int Search(long vo, std::vector<double> &probability, double time_limit);
  
  virtual CostVec _Heuristic(long v, long lid);

  virtual bool _IsPruned(Label& l);

  virtual void _FilterandAddFront(Label& l);

  virtual int _IsTerminal(Label& l);

  virtual void _PostProcRes() override;

  // virtual void InitHeu(long vd) override ; // This is just an example.

  virtual long _GenLabelId() override;

  virtual bool Check_belong(std::vector<long>& visited1, std::vector<long>& visited2);//check if the visited vertices are the same

  virtual void PreProc(PlannerGraph* graph, std::vector<double> &probability);

  virtual void PreProc_k(PlannerGraph* graph, std::vector<double> &probability, int visited_size);

  virtual int Normalization(std::vector<double> &p);

  virtual int Greedy_Search(long vo, std::vector<double> &probability, double time_limit, int type);//type-0:greedy;type-1:greedy combined with cost

  virtual int LKH_Search(std::vector<std::vector<double> > costmatrix, long vo, std::vector<double> &probability, double time_limit);

  virtual int focal_Search(long vo, std::vector<double> &probability, double time_limit, double sub_e);

  virtual int log_Search(long vo, std::vector<double> &probability, double time_limit);

  virtual int _CountUniqueVisited(long id);

  virtual int k_Search(long vo, std::vector<double> &probability, double time_limit, int visited_size);

protected:
  // Add more parameters when needed.
  std::vector< std::vector<long> > _visited; // visited vertices
  std::vector<double> _p; // probability of not finding the target at any vertex 
  std::vector<double> _p_find; // probability of finding the target at any vertex
  std::unordered_map<long, std::vector<long>> _frontier; //map the labels visit the same vertex
  std::vector<std::vector<double>> _heuristic;
  std::set< std::pair<long, long> > _focal;

  HPPPTResult _res;

  long _prune_check_count = 0;
};


/**
 * @brief The entry point of ERCA* (Enhanced Resource Constrained A*).
 * g - a pointer of graph, which can be either of class GridkConn or Roadmap. Polymorphism is used.
 * vo,vd - start and goal nodes.
 * time_limit - the run time limit for search.
 * resource_limits - limit for each type of resource.
 * res - the output argument.
 */
int RunAstarHPPPT(raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res) ;


void SaveResult(raplab::HPPPTResult* res, std::string instance_file_path, bool rewrite);

struct InstanceHPPPTParams
{
  InstanceHPPPTParams() {}
  int dimension = 10;
  int max_size = 500;
  double min_dis = max_size / 100;
  bool if_dpot = false;
  std::pair<double, double> dpot = {0, 0};
};

int Get_Instance(std::vector<std::vector<double> >* output, InstanceHPPPTParams &params);
int Run_Instance(raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;
int Run_Instance_Bayes(raplab::PlannerGraph* g, long vo, std::vector<double> &p, std::vector<double> &p_bayes,
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;
int Run_Greedy(raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;
int Run_HPP(raplab::PlannerGraph* g, std::vector<std::vector<double>> &costmatrix, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;
int Run_Instance_focal(double sub_e, raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;
int Run_Instance_log(raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path) ;  
std::unordered_map<int, std::pair<double, double>> Get_Instance_new (std::vector<std::vector<double> >* output, InstanceHPPPTParams &params);
std::unordered_map<int, std::pair<double, double>> Get_Instance_big (std::vector<std::vector<double> >* output, InstanceHPPPTParams &params);
int Run_Instance_k(raplab::PlannerGraph* g, long vo, std::vector<double> &p, 
  double time_limit, raplab::HPPPTResult* res, std::string file_path, int visited_size) ;

} // end namespace rzq


#endif  // RAPLAB_BASIC_SEARCH_HPP_PT_H_