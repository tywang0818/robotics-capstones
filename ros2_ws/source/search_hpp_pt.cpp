
/*******************************************
 * HPP with Probabilistic Terminals
 * Author: Yunpeng Lyu, Zhongqiang Ren
 * Ownership: RAP-lab@SJTU
 * All Rights Reserved. 
 *******************************************/


#include "search_hpp_pt.hpp"
#include "debug.hpp"
#include <set>
#include <algorithm>
#include <memory>
#include <chrono>
#include <fstream>
#include <sstream>

namespace raplab{

double weight = 1.0;

AstarHPPPT::AstarHPPPT() {};

AstarHPPPT::~AstarHPPPT() {};

int AstarHPPPT::Search(long vo, std::vector<double> &probability, double time_limit) {

  // E.g. if need to override the search, write code here.
   // NOTE: InitHeu(vd) should be called outside

  // _InitFrontiers();
  // _alpha.resize(_graph->NumVertex()); // this requires the graph vertex are numbered from 0 to |V|.

  //Init
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  // _vd = vd;
  _p_find = probability;
  if(P_NORMALIZATION)
    Normalization(_p_find);
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _parent[lo.id] = -1; 
  _visited[lo.id] = std::vector<long>();
  _visited[lo.id].push_back(vo);
  _p[lo.id] = 1;
  auto heur = _Heuristic(vo, lo.id);
  if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### heur = " << heur << std::endl;
  }
  lo.f = lo.g + weight * heur;
  _p[lo.id] = 1-_p_find[vo];
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_HPPPT > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // main search loop
  while ( !_open.empty() ) {

    // check timeout
    if (timer.GetDurationSecond() > time_limit) {
      std::cout << "[INFO] AstarHPPPT::Search timeout !" << std::endl;
      _res.timeout = true;
      break;
    }

    // ## select label l, lexicographic order ##
    Label l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] -------------- pop l begin --------------" << std::endl;
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    // ## lazy dominance check ##
    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### _frontier[" << l.v << "]:" << _frontier[l.v] << std::endl;
    }
    if (_IsPruned(l) ) {
      if (DEBUG_HPPPT > 1) {
        std::cout << "[DEBUG] IsPruned" << std::endl;
      }
      continue;
    }
  _p[lo.id] = 1-_p_find[vo];
    _FilterandAddFront(l);

    if (_IsTerminal(l)) {
      if (DEBUG_HPPPT > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
      }
      break;
    }
    
    // Bayes Filter add to next process
    // _Bayes_Filter(l.id,_p_find);

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    auto cvecs = _graph->GetSuccCosts(l.v);
    if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] -------------- expand l begin --------------" << std::endl;
    }
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      if(!DETECTION_FAILURE_ALLOWED)
      {
        if (std::find(_visited[l.id].begin(), _visited[l.id].end(), u) != _visited[l.id].end()) {
          continue;
        }//check if the vertex has been visited
      }
      CostVec gu = l.g + _p[l.id]*cvecs[idx];
      Label l2(_GenLabelId(), u, gu, zero_f);
      _parent[l2.id] = l.id;
      _visited[l2.id] = _visited[l.id];
      _visited[l2.id].push_back(u);
      _p[l2.id] = _p[l.id];
      auto heur = _Heuristic(u, l2.id);
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ### heur = " << heur << std::endl;
      }
      l2.f = l2.g + weight * heur;
      _p[l2.id] = (1-_p_find[u])*_p[l.id];
      _label[l2.id] = l2;
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] --- Loop v= " << u << " gen l' = " << l2 << std::endl;
        std::cout << "[DEBUG] --- P = " << _p[l2.id] << std::endl;
        std::cout << "[DEBUG] --- visited = " << _visited[l2.id] << std::endl;
      }
      if (_IsPruned(l2) ) {
        if (DEBUG_HPPPT > 1) {
          std::cout << "[DEBUG] ----- IsPruned..." << std::endl;
        }
        continue;
      }
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
      if(DEBUG_HPPPT>2){
        for (const auto& elem : _open) {
          std::cout << elem.first << " " << elem.second << std::endl;
        }
      }
    } // end for
  } // end while
  // ### post-process the results ###
  if(_res.timeout){
    return 0;
  }
  std::cout << "[INFO] AstarHPPPT::_PostProcRes..." << std::endl;
  _PostProcRes();

  _res.n_PruneCheck = _prune_check_count;
  _res.rt_search = timer.GetDurationSecond();

  std::cout << "[INFO] AstarHPPPT::Search exit." << std::endl;
  return 1;
};

HPPPTResult AstarHPPPT::GetHPPPTResult() const {
  return _res;
};

long AstarHPPPT::_GenLabelId() {

  long out = _label_id_gen++;

  if (_label_id_gen > __vec_alloc_total){
    __vec_alloc_total += __vec_alloc_batch;
    _label.resize(__vec_alloc_total);
    _parent.resize(__vec_alloc_total);
    _visited.resize(__vec_alloc_total);
    _p.resize(__vec_alloc_total);
    if (__vec_alloc_batch < __vec_alloc_batch_max){
      __vec_alloc_batch += __vec_alloc_batch;
    }
  }

  return out;
};

CostVec AstarHPPPT::_Heuristic(long v, long lid) {
  CostVec out(_graph->CostDim(), 0);
  if(IF_HEURISTIC){
    // out[0] = _p[lid]*_heuristic[v][(_graph->NumVertex()-1)-_visited[lid].size()];
    // out[0] = _p[lid]*_heuristic[v][(_graph->NumVertex()-1)-_CountUniqueVisited(lid)];

    //tighten heuristic
    CostVec min(_graph->CostDim(), std::numeric_limits<double>::infinity());
    auto succs = _graph->GetSuccs(v);
    auto cvecs = _graph->GetSuccCosts(v);
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      if (std::find(_visited[lid].begin(), _visited[lid].end(), u) != _visited[lid].end()) {
        continue;
      }//check if the vertex has been visited
      auto heuristic_u = CostVec(_graph->CostDim(), 0);
      heuristic_u[0] = _heuristic[u][(_graph->NumVertex()-1)-_CountUniqueVisited(lid)];
      // std::cout<< "u"<<u<<"cost"<<cvecs[idx]<<"total"<<(1-_p_find[v_tem]) * (cvecs[idx]+ sum_cost)<<std::endl;
      if ((cvecs[idx]+ heuristic_u) < min) {
        min = cvecs[idx]+ heuristic_u;
      }
    }
    out[0] = (1-_p_find[v]) * min[0];
    // std::cout<<"id:"<<id<<" extend_times:"<<extend_times<<" min:"<<min[0]<<" v_min:"<<v_min<<std::endl;
  }
  return out;
};

bool AstarHPPPT::_IsPruned(Label& l){
	// check against frontier
  // bool res=(_frontier.find(l.v) != _frontier.end()) ;
  // std::cout << res << std::endl;
  if (_frontier.find(l.v) != _frontier.end()) {
    for (auto& lid: _frontier[l.v]){
      // if(Check_belong(_visited[lid],_visited[l.id]) && _label[lid].g >= l.g){
      //Fixed
      if(Check_belong(_visited[l.id],_visited[lid]) && l.g >= _label[lid].g){
        _prune_check_count++;
        return true;
      }
    }
  }
	return false;
};

bool AstarHPPPT::Check_belong(std::vector<long>& visited1, std::vector<long>& visited2) {
  std::set<long> set_visited2(visited2.begin(), visited2.end());
  for (auto& v : visited1) {
    if (set_visited2.find(v) == set_visited2.end()) {
      return false;
    }
  }
  return true;
}

void AstarHPPPT::_FilterandAddFront(Label& l){
  if (_frontier.find(l.v) != _frontier.end()) {
    std::vector<long> to_be_deleted;
    for (auto& lid: _frontier[l.v]){
      if(Check_belong(_visited[lid],_visited[l.id]) && _label[lid].g >= l.g){
        to_be_deleted.push_back(lid);
        // std::vector<long>& labels = _frontier[l.v];
        // auto iter = std::find(labels.begin(), labels.end(), lid);
        // labels.erase(iter);
      }
    }
    // for loop to_be_deleted, and delete them all from labels[l.v]
    for (auto& lid : to_be_deleted) {
      _frontier[l.v].erase(std::remove(_frontier[l.v].begin(), _frontier[l.v].end(), lid), _frontier[l.v].end());
    }
  }
  _frontier[l.v].push_back(l.id);
	return ;
};

int AstarHPPPT::_IsTerminal(Label& l){
  // if(_visited[l.id].size() ==_graph->NumVertex()-2){//-2 means number of nodes-1
  if(SUB_OPTIMAL){
    if(_p[l.id]<0.001){
      return 2;
    }
  }
  if(_visited[l.id].size() ==_graph->NumVertex()-1){//-1 means number of nodes
    _res.id = l.id;
    _res.cost = l.g;
    return 1;
  }
  
	return 0;
};

void AstarHPPPT::_PostProcRes() {
  // if (_alpha.find(_vd) != _alpha.end()) {
  //   for (auto lid : _alpha[_vd]->label_ids) {
  _res.path = _BuildPath(_res.id);
  //   }
  // // }
  return ;
};

void AstarHPPPT::PreProc(PlannerGraph* graph, std::vector<double> &probability)
{
  _graph = graph;
  _p_find = probability;
  _heuristic.resize(_graph->NumVertex());//vertex from 1 to num_nodes
  // Init
  for (int id = 1; id < _graph->NumVertex(); id++) {
    _heuristic[id].resize(_graph->NumVertex()-1,0);//extend times from 1 to num_nodes-1
  }
  for(int extend_times = 1 ; extend_times < _graph->NumVertex()-1; extend_times++){
    for (int id = 1; id < _graph->NumVertex(); id++){
      CostVec min(_graph->CostDim(), std::numeric_limits<double>::infinity());
      auto succs = _graph->GetSuccs(id);
      auto cvecs = _graph->GetSuccCosts(id);
      for (int idx = 0; idx < succs.size(); idx++) {
        auto u = succs[idx];
        auto heuristic_u = CostVec(_graph->CostDim(), 0);
        heuristic_u[0] = _heuristic[u][extend_times-1];
        // std::cout<< "u"<<u<<"cost"<<cvecs[idx]<<"total"<<(1-_p_find[v_tem]) * (cvecs[idx]+ sum_cost)<<std::endl;
        if ((1-_p_find[id]) * (cvecs[idx]+ heuristic_u) < min) {
          min = (1-_p_find[id]) * (cvecs[idx]+ heuristic_u);
        }
      }
      // std::cout<<"id:"<<id<<" extend_times:"<<extend_times<<" min:"<<min[0]<<" v_min:"<<v_min<<std::endl;
      _heuristic[id][extend_times] = min[0];
    }
  }

  if (DEBUG_HPPPT >3){
    std::cout << "[DEBUG] ### Heuristic Matrix:" << std::endl;
    for (int i = 1; i < _heuristic.size(); i++) {
      for (int j = 1; j < _heuristic[i].size(); j++) {
        std::cout << _heuristic[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
  return ;
}

void AstarHPPPT::PreProc_k(PlannerGraph* graph, std::vector<double> &probability, int visited_size)
{
  // failed
  _graph = graph;
  _p_find = probability;
  _heuristic.resize(graph->NumVertex());//vertex from 1 to num_nodes
  std::vector< std::vector<long> > visited_heuristic; // (id-1) * NumVertex()-2 + extend_times-1
  visited_heuristic.resize((graph->NumVertex()-1)*(graph->NumVertex()-2));
  // Init
  for (int id = 1; id < graph->NumVertex(); id++) {
    _heuristic[id].resize(graph->NumVertex()-1,0);//extend times from 1 to num_nodes-1
  }
  for(int extend_times = 1 ; extend_times < graph->NumVertex()-1; extend_times++){
    for (int id = 1; id < graph->NumVertex(); id++){
      CostVec min(_graph->CostDim(), std::numeric_limits<double>::infinity());
      auto succs = _graph->GetSuccs(id);
      auto cvecs = _graph->GetSuccCosts(id);
      int index;
      int v_tem = 0;
      int index_tem;
      int visited_size_tem = visited_size;
      // prevent k fails
      do{
        for (int idx = 0; idx < succs.size(); idx++) {
          bool if_k_fail = false;
          auto u = succs[idx];
          // check the visited vertices
          if (extend_times>1){
            int index_u = (extend_times-2) * (graph->NumVertex()-1) + (u-1);
            int start = std::max(0, static_cast<int>(visited_heuristic[index_u].size()) - (visited_size_tem - 1));
            // int start = visited_heuristic[index_u].size() - (visited_size - 1);
            if(std::find(visited_heuristic[index_u].begin() + start, visited_heuristic[index_u].end(), id) != visited_heuristic[index_u].end()) {
              continue;
            }
          }
          auto heuristic_u = CostVec(graph->CostDim(), 0);
          heuristic_u[0] = _heuristic[u][extend_times-1];
          // std::cout<< "u"<<u<<"cost"<<cvecs[idx]<<"total"<<(1-_p_find[v_tem]) * (cvecs[idx]+ sum_cost)<<std::endl;
          if ((1-_p_find[id]) * (cvecs[idx]+ heuristic_u) < min) {
            min = (1-_p_find[id]) * (cvecs[idx]+ heuristic_u);
            v_tem = u;
          }
        }
        visited_size_tem--;
      }while(v_tem == 0);
      index = (extend_times-1) * (graph->NumVertex()-1) + (id-1);
      if(extend_times > 1){
        index_tem = (extend_times-2) * (graph->NumVertex()-1) + (v_tem-1);
        visited_heuristic[index] = visited_heuristic[index_tem];
      }
      visited_heuristic[index].push_back(v_tem);
      _heuristic[id][extend_times] = min[0];
    }
    
  }

  if (DEBUG_HPPPT >3){
    std::cout << "[DEBUG] ### Heuristic Matrix:" << std::endl;
    for (int i = 1; i < _heuristic.size(); i++) {
      for (int j = 1; j < _heuristic[i].size(); j++) {
        std::cout << _heuristic[i][j] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << "[DEBUG] ### Visited Heuristic Matrix:" << std::endl;
    for (int i = 0; i < visited_heuristic.size(); i++) {
      std::cout << "Index " << i << ": ";
      int extend_times = i / (graph->NumVertex()-1) + 1;
      int id = i % (graph->NumVertex()-1) + 1;
      std::cout << "extend_times: " << extend_times <<  ", id: " << id << " -> ";
      for (const auto& v : visited_heuristic[i]) {
        std::cout << v << " ";
      }
      std::cout << std::endl;
    }
  }
  return ;
}

int AstarHPPPT::Normalization(std::vector<double> &p){
  //normalize the probability
  double sum = 0;
  for (size_t i = 1; i < p.size(); i++) {
    sum += p[i];
  }
  for (size_t i = 1; i < p.size(); i++) {
    p[i] = p[i]/sum;
  }
  return 1;
}

int AstarHPPPT::Greedy_Search(long vo, std::vector<double> &probability, double time_limit, int type = 0)
{
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  _p_find = probability;
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _label[lo.id] = lo;
  _visited[lo.id] = std::vector<long>();
  _visited[lo.id].push_back(vo); 
  _p[lo.id] = 1;
  _p[lo.id] = 1-_p_find[vo];
  _res.n_generated++;
  if(type == 0)
  {
    //find the vertex with most probablity
    std::vector<std::pair<double, long>> vertex_prob;
    for (size_t i = 0; i < _p_find.size(); ++i) {
      vertex_prob.push_back(std::make_pair(_p_find[i], i));
    }
    std::sort(vertex_prob.begin(), vertex_prob.end(), std::greater<>());

    for (const auto& vp : vertex_prob) {
      if(vp.second == 0)
      break;
      if (std::find(_visited[lo.id].begin(), _visited[lo.id].end(), vp.second) == _visited[lo.id].end()) {
        auto succs = _graph->GetSuccs(lo.v);
        auto cvecs = _graph->GetSuccCosts(lo.v);
        int id = 0;
        if(vp.second>lo.v)
          id = vp.second-2;
        else
          id = vp.second-1;
        lo.g = lo.g + _p[lo.id]*cvecs[id];
        _p[lo.id] = (1-_p_find[vp.second])*_p[lo.id];
        _visited[lo.id].push_back(vp.second);
        lo.v = vp.second;
        _res.n_generated++;
      }
    }
  }
  else
  {
    //find the vertex with minimum distance
    while (1)
    {
      if(_IsTerminal(lo)) {
        
        break;
      }
      auto succs = _graph->GetSuccs(lo.v);
      auto cvecs = _graph->GetSuccCosts(lo.v);
      int id = 0;
      int vertex = 0;
      double min = std::numeric_limits<double>::infinity();
      for (int idx = 0; idx < succs.size(); idx++) {
        auto u = succs[idx];
        if (std::find(_visited[lo.id].begin(), _visited[lo.id].end(), u) != _visited[lo.id].end()) {
          continue;
        }//check if the vertex has been visited
        if (cvecs[idx][0] < min) {
          min = cvecs[idx][0];
          vertex = u;
          id = idx;
        }
      }
      lo.g = lo.g + _p[lo.id]*cvecs[id];
      _p[lo.id] = (1-_p_find[vertex])*_p[lo.id];
      _visited[lo.id].push_back(vertex);
      lo.v = vertex;
      _res.n_generated++;
    }
  }
  _res.id = lo.id;
  _res.cost = lo.g;
  _res.path = _visited[lo.id];
  _res.rt_search = timer.GetDurationSecond();
  return 1;
}

int AstarHPPPT::LKH_Search(std::vector<std::vector<double> > costmatrix, long vo, std::vector<double> &probability, double time_limit)
{
  SimpleTimer timer;
  timer.Start();
  std::cout << "[INFO] lkh_search starts " << std::endl;
  _vo = vo;
  _p_find = probability;
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _label[lo.id] = lo;

  int dimension = _graph->NumVertex()-1;
  bool lkh_finished = false;
  double cost = 0;

  //Set the start
  std::vector<std::vector<double> > transformed_costmatrix;
  transformed_costmatrix.resize(dimension, std::vector<double>(dimension, 0.0));
  for (int i = 0; i < dimension; ++i) { 
      int original_row_index = (i + _vo - 1) % dimension;
      for (int j = 0; j < dimension; ++j) {
          int original_col_index = (j + _vo - 1) % dimension;
          transformed_costmatrix[i][j] = costmatrix[original_row_index][original_col_index];
      }
  }
  costmatrix = transformed_costmatrix;
  
  //HPP->TSP
  for (auto &row : costmatrix) {
      row.push_back(0.0);
  }
  costmatrix.push_back(std::vector<double>(costmatrix[0].size(), 0.0));
  // load lower triangular matrix
    std::ofstream file;
    std::string lkh_tsp = "/home/raplab20/Desktop/HPPPT/src/tsppt_bayes_sim/LKH-2.0.11/test.tsp"; 
    std::cout << lkh_tsp << std::endl;
    file.open(lkh_tsp);
    file << "NAME: test" << std::endl;
    file << "TYPE: TSP" << std::endl;
    file << "DIMENSION: " << dimension + 1 << std::endl;
    file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
    file << "EDGE_WEIGHT_FORMAT: LOWER_DIAG_ROW" << std::endl;
    file << "EDGE_WEIGHT_SECTION" << std::endl;
    for (size_t i = 0; i < costmatrix.size(); i++) {
        for (size_t j = 0; j < costmatrix[i].size(); j++) {
            if (j <= i) {
                file << costmatrix[i][j]<<" ";
            }
            else {
            }
        }
        file << std::endl;
    }
    file << "EOF" << std::endl;
    file.close();


    // lkh search
    std::string lkh = "/home/raplab20/Desktop/HPPPT/src/tsppt_bayes_sim/LKH-2.0.11/LKH";
    std::string command = lkh + " " + "/home/raplab20/Desktop/HPPPT/src/tsppt_bayes_sim/LKH-2.0.11/test.par";
    std::cout << "Running: " << command << std::endl;
    system(command.c_str());
    std::cout << "[INFO] lkh_search ends " << std::endl;
    _res.rt_search = timer.GetDurationSecond();

    // read result
    std::ifstream ifs;
    ifs.open("/home/raplab20/Desktop/HPPPT/src/tsppt_bayes_sim/LKH-2.0.11/result.txt");
	  std::string line;
    bool ispath = false;
    std::vector<long> path;


	while (std::getline(ifs, line)) {
        // Trim leading and trailing whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line == "TOUR_SECTION") {
            ispath = true;
            continue;
        }
        if (ispath) {
            int val = std::stoi(line);
            if (val == dimension + 1) continue;
            if (val == -1) break;
            path.push_back(val);
        }
	}

    //Reduct the path
    std::vector<long> re_path;
    for (int index : path) {
        int original_index = (index -1 + _vo -1) % dimension +1;
        re_path.push_back(original_index);
    }
    path = re_path;

    double pn = 1;
    cost = 0;
    for (int i = 0; i < path.size()-1; i++) {
        pn *= (1-_p_find[path[i]]);
        cost += pn*costmatrix[path[i]-1][path[(i+1)]-1];
    }

    lo.g[0] = cost;
    _res.cost = lo.g;
    _res.path = path;
  
  return 1;
}



// void AstarHPPPT::InitHeu(long vd) {
//   SimpleTimer timer;
//   timer.Start();
//   _dijks.resize(_graph->CostDim());
//   for (size_t i = 0; i<_graph->CostDim(); i++) {
//     _dijks[i].SetGraphPtr(_graph);
//     _dijks[i].ExhaustiveBackwards(vd, std::numeric_limits<double>::infinity(), i) ;
//   }
//   _res.rt_initHeu = timer.GetDurationSecond();

//   _dom_check_count = 0; // reset dom check counter.
//   return ;

// };
void SaveResult(raplab::HPPPTResult* res, std::string instance_file_path, bool rewrite)
{
  std::ofstream file;
  if(!rewrite)
    file.open(instance_file_path, std::ios::app);
  else
    file.open(instance_file_path);
  file << "-----------------------------------" << std::endl;
  file << "NAME: " << res->name << std::endl;
  file << "path nodes = ";
  for (auto& xx : res->path) {
    //if xx is the last element, do not add comma
    if(&xx == &res->path.back())
      file << xx;
    else
      file << xx << ", ";
  }
  file << std::endl;
  file << "cost = " << res->cost[0] << std::endl;
  file << "research time: " << res->rt_search << std::endl;
  file << "number of generated labels: " << res->n_generated << std::endl;
  file << "number of expanded labels: " << res->n_expanded << std::endl;
  file << "number of pruned labels: " << res->n_PruneCheck << std::endl;
  file.close();
}

int RunAstarHPPPT(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res) 
{
  std::string name = res->name;
  size_t cdim = g->CostDim();
  std::cout << "[INFO] RunAstarHPPPT, M=" << cdim << " time_limit = " << time_limit << std::endl;
  int ret_flag = 0;

  auto planner = raplab::AstarHPPPT();
  // planner.SetGraphPtr(g) ; // set graph to the planner
  planner.PreProc(g, p); // pre-process the graph and probability
  
  ret_flag = planner.Search(vo, p, time_limit) ;
  *res = planner.GetHPPPTResult(); // get result
  res->name = name; 
  SaveResult(res, "../../data/result.txt", 0);

  return ret_flag; // TODO, add more return flags.
};


int Get_Instance(std::vector<std::vector<double> >* output, InstanceHPPPTParams &params)
{
  // produce a random instance
  std::unordered_map<int, std::pair<double, double>> coord;
  double x, y;
  bool too_close = false;
  for (int i = 0; i < params.dimension; i++) {
    x = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    y = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    for(int j = 0;j < i;j++){//check if the distance is close
      double dx = x - coord[j].first;
      double dy = y - coord[j].second;
      if(abs(dx) < params.min_dis||abs(dy) < params.min_dis){
        too_close = true;
        break;
      }
    }
    if (too_close){
      i--; // remake vertex i
      too_close = false;
      continue;
    }
    coord[i] = std::make_pair(x, y);
  }
  std::vector<std::vector<double>> costmatrix;
  costmatrix.resize(params.dimension, std::vector<double>(params.dimension, 0.0));
  for (int i = 0; i < params.dimension; i++) {
			for (int j = i; j < params.dimension; j++) {
				double dx = coord[i].first - coord[j].first;
				double dy = coord[i].second - coord[j].second;
				costmatrix[i][j] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
				costmatrix[j][i] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
			}
	}
  *output = costmatrix;

  return 0;
}

int Run_Instance(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  std::string name = res->name;
  size_t cdim = g->CostDim();
  int ret_flag = 0;
  auto planner = raplab::AstarHPPPT();
  planner.PreProc(g, p); // pre-process the graph and probability
  
  ret_flag = planner.Search(vo, p, time_limit) ;
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  // std::string result_path = "../../data/instance_dif_h/" + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, REWRITE);

  return ret_flag;
}

int Run_Instance_Bayes(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, std::vector<double> &p_bayes, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  SimpleTimer timer;
  timer.Start();
  std::string name = res->name;
  std::cout<< "name: "<< res->name << std::endl;
  size_t cdim = g->CostDim();
  int ret_flag = 0;

  HPPPTResult *res_temp = new HPPPTResult();
  long next_vertex = vo;
  double p_true;
  double p_false;
  double normalization_const;
  double p_sensor_true_is_true = p_bayes[0];
  double p_sensor_false_is_false = p_bayes[1];
  double p_sum = 1;
  double cost = 0.0;
  while(1)
  {
    p_sum = p_sum*(1-p[next_vertex]);
    //Bayes_Filter
    p_true = (1-p_sensor_true_is_true)*p[next_vertex];
    p_false = p_sensor_false_is_false*(1-p[next_vertex]);
    normalization_const = 1/(p_true+p_false);
    p[next_vertex] = p_true*normalization_const;

    auto planner = raplab::AstarHPPPT();
    planner.PreProc(g, p); // pre-process the graph and probability
    ret_flag = planner.Search(next_vertex, p, time_limit) ;
    *res_temp = planner.GetHPPPTResult(); // get result

    cost += p_sum * g->GetCost(next_vertex, res_temp->path[1])[0];
    res->n_expanded += res_temp->n_expanded;
    res->n_generated += res_temp->n_generated;
    res->n_PruneCheck += res_temp->n_PruneCheck;
    res->path.push_back(next_vertex);
    next_vertex = res_temp->path[1];
    res->cost=CostVec(g->CostDim(), 0);
    std::cout<<"p:"<<p<<std::endl;  
    std::cout<< "path:"<<res->path<<std::endl;
    if(res->path.size()>20 )
      break;
  }
  res->name = name;
  res->cost.resize(cdim);
  res->cost[0] = cost;
  res->rt_search = timer.GetDurationSecond();
  std::cout<< "cost:"<<res->cost<<std::endl;
  std::string result_path = file_path + name + ".txt"; 
  if(!res->timeout)
      SaveResult(res, result_path, REWRITE);
  return ret_flag;
}

int Run_Greedy(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  std::string name = res->name;
  std::vector< std::vector<long> > visited; // visited vertices
  size_t cdim = g->CostDim();
  int ret_flag = 0;

  auto planner = raplab::AstarHPPPT();
  planner.SetGraphPtr(g);
  ret_flag = planner.Greedy_Search(vo, p, time_limit, 1);
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, 0);

  return ret_flag;
}

std::unordered_map<int, std::pair<double, double>> Get_Instance_new(std::vector<std::vector<double> >* output, InstanceHPPPTParams &params)
{
  // produce a random instance
  std::unordered_map<int, std::pair<double, double>> coord;
  double x, y;
  bool too_close = false;
  // if (params.dimension > 40) params.max_size = 1000;
  if (params.dimension > 40) params.max_size = 5000;
  for (int i = 0; i < params.dimension; i++) {
    //set depot
    if(params.if_dpot && i == 0){
      x = params.dpot.first;
      y = params.dpot.second;
      continue;
    }
    x = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    y = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    for(int j = 0 ; j < i;j++){//check if the distance is close
      
      double dx = x - coord[j].first;
      double dy = y - coord[j].second;

      if(abs(dx) < params.min_dis||abs(dy) < params.min_dis){
        too_close = true;
        break;
      }
    }
    if (too_close){
      i--; // remake vertex i
      too_close = false;
      continue;
    }
    coord[i] = std::make_pair(x, y);
  }
  std::vector<std::vector<double>> costmatrix;
  costmatrix.resize(params.dimension, std::vector<double>(params.dimension, 0.0));
  for (int i = 0; i < params.dimension; i++) {
			for (int j = i; j < params.dimension; j++) {
				double dx = coord[i].first - coord[j].first;
				double dy = coord[i].second - coord[j].second;
				costmatrix[i][j] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
				costmatrix[j][i] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
			}
	}
  *output = costmatrix;

  return coord;
}

std::unordered_map<int, std::pair<double, double>> Get_Instance_big(std::vector<std::vector<double> >* output, InstanceHPPPTParams &params)
{
  // produce a random instance
  std::unordered_map<int, std::pair<double, double>> coord;
  double x, y;
  bool too_close = false;
  params.max_size = 10 * params.dimension;
  params.min_dis = params.dimension / 100;
  for (int i = 0; i < params.dimension; i++) {
    //set depot
    if(params.if_dpot && i == 0){
      x = params.dpot.first;
      y = params.dpot.second;
      continue;
    }
    x = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    y = static_cast<double>(rand()) / RAND_MAX * params.max_size;
    for(int j = 0 ; j < i;j++){//check if the distance is close
      
      double dx = x - coord[j].first;
      double dy = y - coord[j].second;

      if(abs(dx) < params.min_dis||abs(dy) < params.min_dis){
        too_close = true;
        break;
      }
    }
    if (too_close){
      i--; // remake vertex i
      too_close = false;
      continue;
    }
    coord[i] = std::make_pair(x, y);
  }
  std::vector<std::vector<double>> costmatrix;
  costmatrix.resize(params.dimension, std::vector<double>(params.dimension, 0.0));
  for (int i = 0; i < params.dimension; i++) {
			for (int j = i; j < params.dimension; j++) {
				double dx = coord[i].first - coord[j].first;
				double dy = coord[i].second - coord[j].second;
				costmatrix[i][j] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
				costmatrix[j][i] = static_cast<int>(sqrt(dx * dx + dy * dy) + 0.5);
			}
	}
  *output = costmatrix;

  return coord;
}

int Run_HPP(
  raplab::PlannerGraph* g, std::vector<std::vector<double>> &costmatrix, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  std::string name = res->name;
  std::vector< std::vector<long> > visited; // visited vertices
  int ret_flag = 0;

  auto planner = raplab::AstarHPPPT();
  planner.SetGraphPtr(g);
  ret_flag = planner.LKH_Search(costmatrix, vo, p, time_limit);
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, 1);

  return ret_flag;
}

int AstarHPPPT::_CountUniqueVisited(long id) {
  std::set<long> unique_elements(_visited[id].begin(), _visited[id].end());
  return unique_elements.size();
}

int AstarHPPPT::focal_Search(long vo, std::vector<double> &probability, double time_limit, double sub_e = 1) {

  // _InitFrontiers();
  // _alpha.resize(_graph->NumVertex()); // this requires the graph vertex are numbered from 0 to |V|.

  //Init
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  _p_find = probability;
  if(P_NORMALIZATION)
    Normalization(_p_find);
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _parent[lo.id] = -1; 
  _visited[lo.id] = std::vector<long>();
  _visited[lo.id].push_back(vo);
  _p[lo.id] = 1;
  auto heur = _Heuristic(vo, lo.id);
  if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### heur = " << heur << std::endl;
  }
  lo.f = lo.g + weight * heur;
  _p[lo.id] = 1-_p_find[vo];
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );
  if (DEBUG_HPPPT > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }
  // main search loop
  while ( !_open.empty() ) {
    if (DEBUG_HPPPT > 0) {
      for (const auto& elem : _open) {
        std::cout << "f: ";
        for (const auto& val : elem.first) {
          std::cout << val << " ";
        }
        std::cout << ", id: " << elem.second << std::endl;
      }
    }
    // check timeout
    if (timer.GetDurationSecond() > time_limit) {
      std::cout << "[INFO] AstarHPPPT::Search timeout !" << std::endl;
      _res.timeout = true;
      break;
    }
    //select l with f less than the 1+sub_e times of the best f in the open list
    for (const auto& item : _open) {
      if (item.first < (1 + sub_e) * _open.begin()->first) {
        _focal.insert(std::make_pair(_CountUniqueVisited(item.second), item.second) );
      } 
      else {
        break;
      }
    }
    // ## select label l, lexicographic order with most visited vertices ##
    Label l = _label[_focal.rbegin()->second];
    // Find the element in _open with second == l.id and erase it
    for (auto it = _open.begin(); it != _open.end(); ++it) {
      if (it->second == l.id) {
        _open.erase(it);
        break;
      }
    }
    _focal.clear();

    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] -------------- pop l begin --------------" << std::endl;
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }
    // ## lazy dominance check ##
    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### _frontier[" << l.v << "]:" << _frontier[l.v] << std::endl;
    }
    if (_IsPruned(l) ) {
      if (DEBUG_HPPPT > 1) {
        std::cout << "[DEBUG] IsPruned" << std::endl;
      }
      continue;
    }

    _FilterandAddFront(l);

    if (_IsTerminal(l)) {
      if (DEBUG_HPPPT > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
      }
      break;
    }
    // Bayes Filter add to next process
    // _Bayes_Filter(l.id,_p_find);

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    auto cvecs = _graph->GetSuccCosts(l.v);
    if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] -------------- expand l begin --------------" << std::endl;
    }
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      if(!DETECTION_FAILURE_ALLOWED)
      {
        if (std::find(_visited[l.id].begin(), _visited[l.id].end(), u) != _visited[l.id].end()) {
          continue;
        }//check if the vertex has been visited
      }
      CostVec gu = l.g + _p[l.id]*cvecs[idx];
      Label l2(_GenLabelId(), u, gu, zero_f);
      _parent[l2.id] = l.id;
      _visited[l2.id] = _visited[l.id];
      _visited[l2.id].push_back(u);
      _p[l2.id] = _p[l.id];
      auto heur = _Heuristic(u, l2.id);
      l2.f = l2.g + weight * heur;
      _p[l2.id] = (1-_p_find[u])*_p[l.id];
      _label[l2.id] = l2;
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] --- Loop v= " << u << " gen l' = " << l2 << std::endl;
        std::cout << "[DEBUG] --- P = " << _p[l2.id] << std::endl;
        std::cout << "[DEBUG] --- visited = " << _visited[l2.id] << std::endl;
      }
      if (_IsPruned(l2) ) {
        if (DEBUG_HPPPT > 1) {
          std::cout << "[DEBUG] ----- IsPruned..." << std::endl;
        }
        continue;
      }
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
      if(DEBUG_HPPPT>2){
        for (const auto& elem : _open) {
          std::cout << elem.first << " " << elem.second << std::endl;
        }
      }
    } // end for
  } // end while
  // ### post-process the results ###
  if(_res.timeout){
    return 0;
  }
  std::cout << "[INFO] AstarHPPPT::_PostProcRes..." << std::endl;
  _PostProcRes();

  _res.n_PruneCheck = _prune_check_count;
  _res.rt_search = timer.GetDurationSecond();

  std::cout << "[INFO] AstarHPPPT::Search exit." << std::endl;
  return 1;
};

int Run_Instance_focal(
  double sub_e, raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  std::string name = res->name;
  size_t cdim = g->CostDim();
  int ret_flag = 0;
  auto planner = raplab::AstarHPPPT();
  planner.PreProc(g, p); // pre-process the graph and probability
  
  ret_flag = planner.focal_Search(vo, p, time_limit, sub_e) ;
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  // std::string result_path = "../../data/instance_dif_h/" + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, REWRITE);

  return ret_flag;
}

int AstarHPPPT::log_Search(long vo, std::vector<double> &probability, double time_limit) {

  //Init
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  // _vd = vd;
  _p_find = probability;
  if(P_NORMALIZATION)
    Normalization(_p_find);
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _parent[lo.id] = -1; 
  _visited[lo.id] = std::vector<long>();
  _visited[lo.id].push_back(vo);
  _p[lo.id] = log(1);
  auto heur = _Heuristic(vo, lo.id);
  if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### heur = " << heur << std::endl;
  }
  lo.f = lo.g + weight * heur;
  _label[lo.id] = lo;
  _p[lo.id] = log(1-_p_find[vo]);
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_HPPPT > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // main search loop
  while ( !_open.empty() ) {

    // check timeout
    if (timer.GetDurationSecond() > time_limit) {
      std::cout << "[INFO] AstarHPPPT::Search timeout !" << std::endl;
      _res.timeout = true;
      break;
    }

    // ## select label l, lexicographic order ##
    Label l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] -------------- pop l begin --------------" << std::endl;
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    // ## lazy dominance check ##
    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### _frontier[" << l.v << "]:" << _frontier[l.v] << std::endl;
    }
    if (_IsPruned(l) ) {
      if (DEBUG_HPPPT > 1) {
        std::cout << "[DEBUG] IsPruned" << std::endl;
      }
      continue;
    }

    _FilterandAddFront(l);

    if (_IsTerminal(l)) {
      if (DEBUG_HPPPT > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
      }
      break;
    }
    
    // Bayes Filter add to next process
    // _Bayes_Filter(l.id,_p_find);

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    auto cvecs = _graph->GetSuccCosts(l.v);
    if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] -------------- expand l begin --------------" << std::endl;
    }
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      if(!DETECTION_FAILURE_ALLOWED)
      {
        if (std::find(_visited[l.id].begin(), _visited[l.id].end(), u) != _visited[l.id].end()) {
          continue;
        }//check if the vertex has been visited
      }
      CostVec gu = l.g + exp(_p[l.id])*cvecs[idx];
      Label l2(_GenLabelId(), u, gu, zero_f);
      _parent[l2.id] = l.id;
      auto heur = _Heuristic(u, l2.id);
      l2.f = l2.g + weight * heur;
      _label[l2.id] = l2;
      _p[l2.id] = log(1-_p_find[u])+_p[l.id];
      _visited[l2.id] = _visited[l.id];
      _visited[l2.id].push_back(u);
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] --- Loop v= " << u << " gen l' = " << l2 << std::endl;
        std::cout << "[DEBUG] --- P = " << _p[l2.id] << std::endl;
        std::cout << "[DEBUG] --- visited = " << _visited[l2.id] << std::endl;
      }
      if (_IsPruned(l2) ) {
        if (DEBUG_HPPPT > 1) {
          std::cout << "[DEBUG] ----- IsPruned..." << std::endl;
        }
        continue;
      }
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
      if(DEBUG_HPPPT>2){
        for (const auto& elem : _open) {
          std::cout << elem.first << " " << elem.second << std::endl;
        }
      }
    } // end for
  } // end while
  // ### post-process the results ###
  if(_res.timeout){
    return 0;
  }
  std::cout << "[INFO] AstarHPPPT::_PostProcRes..." << std::endl;
  _PostProcRes();

  _res.n_PruneCheck = _prune_check_count;
  _res.rt_search = timer.GetDurationSecond();

  std::cout << "[INFO] AstarHPPPT::Search exit." << std::endl;
  return 1;
};

int Run_Instance_log(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path)
{
  std::string name = res->name;
  size_t cdim = g->CostDim();
  int ret_flag = 0;
  auto planner = raplab::AstarHPPPT();
  planner.PreProc(g, p); // pre-process the graph and probability
  
  ret_flag = planner.log_Search(vo, p, time_limit) ;
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  // std::string result_path = "../../data/instance_dif_h/" + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, REWRITE);

  return ret_flag;
}

int Run_Instance_k(
  raplab::PlannerGraph* g, long vo, std::vector<double> &p, double time_limit, raplab::HPPPTResult* res, std::string file_path, int visited_size)
{
  std::string name = res->name;
  size_t cdim = g->CostDim();
  int ret_flag = 0;
  auto planner = raplab::AstarHPPPT();
  planner.PreProc_k(g, p, visited_size); // pre-process the graph and probability
  
  ret_flag = planner.Search(vo, p, time_limit) ;
  *res = planner.GetHPPPTResult(); // get result
  res->name = name;
  std::string result_path = file_path + name + ".txt"; 
  // std::string result_path = "../../data/instance_dif_h/" + name + ".txt"; 
  if(!res->timeout)
    SaveResult(res, result_path, REWRITE);

  return ret_flag;
}

int AstarHPPPT::k_Search(long vo, std::vector<double> &probability, double time_limit, int visited_size) {
// add k vertices visited record
// need to add constraint
  //Init
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  _p_find = probability;
  if(P_NORMALIZATION)
    Normalization(_p_find);
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  auto zero_f = CostVec(_graph->CostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, zero_f);
  _parent[lo.id] = -1; 
  _visited[lo.id] = std::vector<long>();
  _visited[lo.id].push_back(vo);
  _p[lo.id] = 1;
  auto heur = _Heuristic(vo, lo.id);
  if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### heur = " << heur << std::endl;
  }
  lo.f = lo.g + weight * heur;
  _p[lo.id] = 1-_p_find[vo];
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_HPPPT > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // main search loop
  while ( !_open.empty() ) {

    // check timeout
    if (timer.GetDurationSecond() > time_limit) {
      std::cout << "[INFO] AstarHPPPT::Search timeout !" << std::endl;
      _res.timeout = true;
      break;
    }

    // ## select label l, lexicographic order ##
    Label l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] -------------- pop l begin --------------" << std::endl;
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    // ## lazy dominance check ##
    if (DEBUG_HPPPT > 0) {
      std::cout << "[DEBUG] ### _frontier[" << l.v << "]:" << _frontier[l.v] << std::endl;
    }
    if (_IsPruned(l) ) {
      if (DEBUG_HPPPT > 1) {
        std::cout << "[DEBUG] IsPruned" << std::endl;
      }
      continue;
    }
  _p[lo.id] = 1-_p_find[vo];
    _FilterandAddFront(l);

    if (_IsTerminal(l)) {
      if (DEBUG_HPPPT > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
      }
      break;
    }
    
    // Bayes Filter add to next process
    // _Bayes_Filter(l.id,_p_find);

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    auto cvecs = _graph->GetSuccCosts(l.v);
    if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] -------------- expand l begin --------------" << std::endl;
    }
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      if(!DETECTION_FAILURE_ALLOWED)
      {
        if (std::find(_visited[l.id].begin(), _visited[l.id].end(), u) != _visited[l.id].end()) {
          continue;
        }//check if the vertex has been visited
      }
      CostVec gu = l.g + _p[l.id]*cvecs[idx];
      Label l2(_GenLabelId(), u, gu, zero_f);
      _parent[l2.id] = l.id;
      _visited[l2.id] = _visited[l.id];
      _visited[l2.id].push_back(u);
      _p[l2.id] = _p[l.id];
      auto heur = _Heuristic(u, l2.id);
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ### heur = " << heur << std::endl;
      }
      l2.f = l2.g + weight * heur;
      _p[l2.id] = (1-_p_find[u])*_p[l.id];
      _label[l2.id] = l2;
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] --- Loop v= " << u << " gen l' = " << l2 << std::endl;
        std::cout << "[DEBUG] --- P = " << _p[l2.id] << std::endl;
        std::cout << "[DEBUG] --- visited = " << _visited[l2.id] << std::endl;
      }
      if (_IsPruned(l2) ) {
        if (DEBUG_HPPPT > 1) {
          std::cout << "[DEBUG] ----- IsPruned..." << std::endl;
        }
        continue;
      }
      if (DEBUG_HPPPT > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
      if(DEBUG_HPPPT>2){
        for (const auto& elem : _open) {
          std::cout << elem.first << " " << elem.second << std::endl;
        }
      }
    } // end for
  } // end while
  // ### post-process the results ###
  if(_res.timeout){
    return 0;
  }
  std::cout << "[INFO] AstarHPPPT::_PostProcRes..." << std::endl;
  _PostProcRes();

  _res.n_PruneCheck = _prune_check_count;
  _res.rt_search = timer.GetDurationSecond();

  std::cout << "[INFO] AstarHPPPT::Search exit." << std::endl;
  return 1;
};


} // end namespace rzq