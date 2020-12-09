/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"


CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x((CParams::WindowWidth / CParams::iGridCellDim) + 1),
	_grid_size_y((CParams::WindowHeight / CParams::iGridCellDim) + 1)
{
}
/**
 The initialize method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	//TODO
   srand(time(NULL));
   Rewards = new double * [41];
   for(int i=0; i<41; ++i)
      Rewards[i] = new double[41];
      
   for(int i=0; i<41; ++i)
      for(int j=0; j<41; ++j)
         Rewards[i][j] = 0;
   
   Q = new double *** [CParams::iNumSweepers];
   for(int i=0; i<CParams::iNumSweepers; ++i){
      Q[i] = new double ** [_grid_size_x];
      for(int j=0; j<_grid_size_x; ++j){
         Q[i][j] = new double * [_grid_size_y];
         for(int k=0; k<_grid_size_y; ++k){
            Q[i][j][k] = new double [4];
            for(int l=0; l<4; ++l)
               Q[i][j][k][l] = 0; 
         }
      }
   }
}
/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
         int closestObj  = m_vecSweepers[sweeper_no]->CheckForObject(m_vecObjects, CParams::dMineScale);
         for(int i=0; i<41; ++i){
            for(int j=0; j<41; ++j){
               for(int k=0; k<m_vecObjects.size(); ++k){
                  SVector2D<int> position =  m_vecObjects[k]->getPosition();
                  if(!((position.x==i) && (position.y==j)))
                     continue;
                  else if(m_vecObjects[k]->isDead())
                     Rewards[i][j] = -5;
                  else{
                     switch(m_vecObjects[k]->getType()){
                     case CDiscCollisionObject::Mine:
                        {
                        Rewards[i][j] = 10;
                        }
                     case CDiscCollisionObject::Rock:
                        {
                        Rewards[i][j] = -10;
                        }
                     case CDiscCollisionObject::SuperMine:
                        {
                        Rewards[i][j] = -10;
                        }
                     }
                  }
               }
            }
         }
         return -10;       
}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update(void)
{

   if(m_iIterations == 51){
      ofstream of;
      of.open("results.txt");
      int totalG=0; int totalD=0; int mostG=0;
      for(int i=0; i<50; ++i){
         of<< m_vecMostMinesGathered[i] << " " << m_vecDeaths[i] << std::endl;
         totalG+= m_vecMostMinesGathered[i];
         totalD+= m_vecDeaths[i];
         if(m_vecAvMinesGathered[i] > mostG)
            mostG = m_vecAvMinesGathered[i];
      }
      double aveGathered = double(totalG)/50;
      double aveDeaths = double(totalD) /50;
      of<<std::endl;
      of<<mostG << " " << aveGathered << " " << aveDeaths;
      of.close();
   }
	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
   
   //gets a count of all dead sweepers
	uint cDead = std::count_if(m_vecSweepers.begin(),
							   m_vecSweepers.end(),
						       [](CDiscMinesweeper * s)->bool{
								return s->isDead();
							   });
	if (cDead == CParams::iNumSweepers){
		printf("All dead ... skipping to next iteration\n");
		m_iTicks = CParams::iNumTicks;
	}
   
   int * actions  = new int[CParams::iNumSweepers];
	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		//1:::Observe the current state:
      SVector2D<int> currentState (m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y);
      //2:::Select action with highest historic return:
      int a = rand()%3;
      if(Q[sw][currentState.x/10][currentState.y/10][1] > Q[sw][currentState.x/10][currentState.y/10][a])
      int a = 1;
      if(Q[sw][currentState.x/10][currentState.y/10][3] > Q[sw][currentState.x/10][currentState.y/10][a])
         a = 3;
      if(Q[sw][currentState.x/10][currentState.y/10][0] > Q[sw][currentState.x/10][currentState.y/10][a])
         a = 0;
      if(Q[sw][currentState.x/10][currentState.y/10][2] > Q[sw][currentState.x/10][currentState.y/10][a])
         a = 2;
      if((Q[sw][currentState.x/10][currentState.y/10][0]==Q[sw][currentState.x/10][currentState.y/10][1])&&(Q[sw][currentState.x/10][currentState.y/10][1]==Q[sw][currentState.x/10][currentState.y/10][2])&&(Q[sw][currentState.x/10][currentState.y/10][2]==Q[sw][currentState.x/10][currentState.y/10][3]))
        // a = rand()%3;
      m_vecSweepers[sw]->setRotation((ROTATION_DIRECTION)a);
      actions[sw] = a;
		//now call the parents update, so all the sweepers fulfill their chosen action
	}
	
	CDiscController::Update(); //call the parent's class update. Do not delete this.
	
	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
		//3:::Observe new state:
		SVector2D<int> newState (m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y);
		//4:::Update _Q_s_a accordingly:
      /*double old_value = Q[sw][m_vecSweepers[sw]->PrevPosition().x/10][m_vecSweepers[sw]->PrevPosition().y/10][actions[sw]];
      double n = 0.9;
      R(m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y, sw);
      double r = Rewards[newState.x/10][newState.y/10];
      double y = 0.6;
      SVector2D<int> currentState (m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y);
      int a = 1;
      if(Q[sw][currentState.x/10][currentState.y/10][3] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 3;
      if(Q[sw][currentState.x/10][currentState.y/10][0] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 0;
      if(Q[sw][currentState.x/10][currentState.y/10][2] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 2;*/
		//Q[sw][m_vecSweepers[sw]->PrevPosition().x/10][m_vecSweepers[sw]->PrevPosition().y/10][actions[sw]] = old_value + 0.9*(r + y*a - old_value);*/
	   R(m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y, sw);
      double old_value = Rewards[m_vecSweepers[sw]->PrevPosition().x/10][m_vecSweepers[sw]->PrevPosition().y/10];
      double n = 0.6;
      double r = Rewards[newState.x/10][newState.y/10];
      double y = 0.6;
      SVector2D<int> currentState (m_vecSweepers[sw]->Position().x, m_vecSweepers[sw]->Position().y);
      int a = 1;
      if(Q[sw][currentState.x/10][currentState.y/10][3] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 3;
      if(Q[sw][currentState.x/10][currentState.y/10][0] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 0;
      if(Q[sw][currentState.x/10][currentState.y/10][2] >= Q[sw][currentState.x/10][currentState.y/10][a])
         a = 2;
      Q[sw][m_vecSweepers[sw]->PrevPosition().x/10][m_vecSweepers[sw]->PrevPosition().y/10][a] = old_value + 0.9*(r + y*a - old_value);
   }
	return true;
}

CQLearningController::~CQLearningController(void)
{
	//TODO: dealloc stuff here if you need to
	   for(int i=0; i<41; ++i)
      Rewards[i] = new double[41];
      
   for(int i=0; i<41; ++i){ 	
      for(int j=0; j<41; ++j)
         delete [] Rewards[i];
      delete [] Rewards;
   }
   
   Q = new double *** [CParams::iNumSweepers];
   for(int i=0; i<CParams::iNumSweepers; ++i){
      for(int j=0; j<_grid_size_x; ++j){
         for(int k=0; k<_grid_size_y; ++k){
            delete [] Q[i][j] ;
         }
	 delete [] Q[i];
      }
      delete [] Q;
   }	
}
