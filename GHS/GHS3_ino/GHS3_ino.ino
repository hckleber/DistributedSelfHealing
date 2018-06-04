

#include <variables.h>
#include <behaviors.h>
#include <nRF24L01.h>
#include <sensors.h>
#include <speed_control.h>
#include <utility.h>
#include <io.h>
#include <interrupt.h>
#include <stdlib.h>

/* Testes Iniciais*/
void StarTest (void){
  int AddrId[4] = {3215, 3219};
  int temp = 0;
  int i = 0;    

  //Checa o ID do robô
  while(temp<30000){
      for(i=0;i<2;i++){
    if(rfAddress == AddrId[i]){
      turnOnGreenLeds();
      pwm_blue = MAX_LEDS_PWM; updateRedLed(pwm_red); updateGreenLed(0);  updateBlueLed(pwm_blue);
    }else{
      pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(0);  updateBlueLed(pwm_blue);
    }
   }
    temp++;
  }
  
  turnOffGreenLeds();

}
  
  


/* Função de Inicialização */
void Inicializa(void){
  // Cria matriz de vizinhos
  memset(vizinhos[0], 0, sizeof(vizinhos[0])); //inicializa "vizinhos[0]" com valores iniciais 
  vizinhos[0][0] = 0; // inicializa os status de arestas
  vizinhos[1][0] = 2; // inicializa os pesos de cada vizinho
  
  if(rfAddress == 3215){
  vizinhos[2][0] = 3219; // inicializa os endereços dos vizinhos 
  }else{
  vizinhos[2][0] = 3215;
  }
  
  // Led Branco para Sleeping
  pwm_red = MAX_LEDS_PWM; updateRedLed(pwm_red);  updateBlueLed(0); updateGreenLed(0);  
  
}

/* Despertar espontâneo */
void Despertar (void){
  if(rfAddress == 3219){
    Wakeup();
  }  

}

/* Processo de despertar */
void Wakeup (void){
  int i = 0;
  // Verifica qual a aresta de peso minimo
  m_peso = vizinhos[1][0];
  m_pos = 0;
  
  for (i=1;i<sizeof(vizinhos[1]);i++){
    if(vizinhos[1][i] < m_peso){
      m_peso = vizinhos[1][i];
      m_pos = i;
    }
  }
  ////////////////////////////////////////
  vizinhos[0][m_pos] = 1;
  LN = 0;
  SN = 2;
  Fc = 0;
    
  /* Preparando envio da msg */  
  Tp_Msg = 1;
  L = LN;
  F = 0;
  S = 0;
  Bw = 0;
  Dest = vizinhos[2][m_pos];
  // Envia msg Connect(LN) 
  
  // Led Laranja para Found
  pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(0);  updateBlueLed(pwm_blue); 
  

 }


/* Resposta para a mensagem Connect */
void R_Connect (void){
  int i = 0;
  int j = 0;
  int a = 0;
  //int b = 0;
  
  if(Fila[F_Ini][2] < LN){
  for(i=0;i<sizeof(vizinhos[1]);i++){
    if(vizinhos[2][i] == Dest){
      vizinhos[0][i] = 1;
    }
  }
  /* Preparando envio da msg */
  Tp_Msg = 2;
  L = LN;
  F = FN;
  S = SN;
  Bw = 0;
  Dest = Fila[F_Ini][7];
  // Envia msg Initiate(LN, FN, SN)  para o nó vizinho
    if(SN == 1){
      Fc++;
    }
  }else{
     for(i=0;i<sizeof(vizinhos[1]);i++){
       if(vizinhos[2][i] == Fila[F_Ini][6]){
         if(vizinhos[0][i] == 0){
           a = 1;
         }else{
           F = vizinhos[1][i];
           Dest = vizinhos[2][i];           
         }
       }
       i=sizeof(vizinhos[1]);
     }
       if(a==1){
         EndFila();
         Dest = 0;
       }else{
         /* Preparando envio da msg */
         Tp_Msg = 2;
         L = LN + 1;
         //F => Está configurado no loop for acima
         S = 1;
         Bw = 0;
         //Dest => Está configurado no loop for acima
         // Envia msg Initiate(LN+1, w(j), Find)  para o nó vizinho
       }
     }
  
}
  
void R_Initiate(void) {
 // Armazena Msg no buffer
 if(F_Initiate==0){
//   int i = 0;
//   for(i=0;i<8;i++){
//     Msg_Buffer[0][i] = Fila[F_Ini][i];
//   }
   LN = Fila[F_Ini][2];
   FN = Fila[F_Ini][3];
   SN = Fila[F_Ini][4];
   Ib = Fila[F_Ini][6];
   Be = 0;
   BwR = 99; // 32767
   
   L = LN;
   F = FN;
   S = SN;
   
   F_Initiate = 1;
   //Add_Vizin = 0;
 }
 
 if(Add_Vizin<NB_VIZIN){
   if(vizinhos[2][Add_Vizin] != Ib && vizinhos[0][Add_Vizin]==1){
     Dest = vizinhos[2][Add_Vizin];
     if(SN==1){
       Fc++;
     }
   }else{
     Add_Vizin++;
   }
 }else{
   if(SN==1){
     F_Initiate = 0;
     Add_Vizin++;     
     P_Test();
     // Vermelho
     //pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(pwm_green);  updateBlueLed(pwm_blue);
   }
 }
}


void P_Test(void){
  int i = 0;
  int a = 0;
  int m_peso = 0;
  
  for(i=0;i<NB_VIZIN;i++){
   if(vizinhos[0][i] == 0 && a == 0){
     Te = vizinhos[1][i];
     Dest = vizinhos[2][i];
     a = 1;
   }else{
     if(vizinhos[0][i] == 0 && vizinhos[1][i] < Te){
       Te = vizinhos[1][i];
       Dest = vizinhos[2][i];
     }
   }
  }
  
  if(a == 1){
    /* Preparando envio da msg */
    Tp_Msg = 3;
    L = LN;
    F = FN;
    S = 0;
    Bw = 0;
    //Dest => Está configurado no loop for acima
    //Envia msg Test(LN, FN)
  }else{
    Te = 0;
    // Executa processo Report
    P_Report();
    //Vermelho
    //pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(pwm_green);  updateBlueLed(pwm_blue);
  }
   
}

void R_Test (void){
  if(Fila[F_Ini][2]>LN){
    EndFila();
  }else{
    if(Fila[F_Ini][3]!=FN){
      /* Preparando envio da msg */
      Tp_Msg = 6;
      L = 0;
      F = 0;
      S = 0;
      Bw = 0;
      Dest = Fila[F_Ini][6];
      //Envia msg Accept
    }else{
      int i = 0;
      for(i=0;i<NB_VIZIN;i++){
       if(vizinhos[2][i] == Fila[F_Ini][6] && vizinhos[0][i] == 0){
         vizinhos[0][i] == 2;
       }
       if(Te!=Fila[F_Ini][6]){
         /* Preparando envio da msg */
         Tp_Msg = 5;
         L = 0;
         F = 0;
         S = 0;
         Bw = 0;
         Dest = Fila[F_Ini][6];
         //Envia msg Rejected
       }else{
         P_Test();
       }
      }
    }
  }
}

void P_Report (void){
 if(Fc == 0 && Te == 0){
  SN = 2;
  Tp_Msg = 4;
  L = 0; 
  F = 0;
  S = 0;
  Bw = BwR;
  Dest = Ib;
  //Vermelho
  pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(pwm_green);  updateBlueLed(pwm_blue);
 }
}

void R_Report (void){
 if(Fila[F_Ini][6] != Ib){
  Fc--;
  if(Fila[F_Ini][5]<BwR){
   BwR = Fila[F_Ini][5];
   Be = Fila[F_Ini][6];
   P_Report();
  }
 }else{
  if(SN==1){
    EndFila();
  }else{
   if(Fila[F_Ini][5]>BwR){
    P_Change();
    //Rosa
    pwm_blue = MAX_LEDS_PWM; updateRedLed(0); updateGreenLed(pwm_blue);  updateBlueLed(70);
   }else{
     if(Fila[F_Ini][5]==BwR && Fila[F_Ini][5]==99){
       Halt = 55; // Para o envio e recebimento de msg pelo Robô
       //Cinza
       pwm_blue = MAX_LEDS_PWM; updateRedLed(80); updateGreenLed(40);  updateBlueLed(40);
     }
   }
  }
 }
}

void R_Accept (void){
 int i = 0;
 Te = 0;
 for(i=0;i<NB_VIZIN;i++){
   if(vizinhos[2][i] == Fila[F_Ini][6] && vizinhos[1][i] < BwR){
     Be = Fila[F_Ini][6];
     BwR = vizinhos[1][i];
     i = NB_VIZIN;
   }
 }
 P_Report();
}

void R_Rejected (void){
 int i = 0;
 for(i=0;i<NB_VIZIN;i++){
   if(vizinhos[2][i] == Fila[F_Ini][6] && vizinhos[0][i] == 0){
     vizinhos[0][i] = 2;
     i = NB_VIZIN;
   }
 }
 P_Test();
}

void P_Change (void){
 int i = 0;
 int flag = 0;
 for(i=0;i<NB_VIZIN;i++){
   if(vizinhos[2][i] == Be && vizinhos[0][i] == 1){
     /* Preparando envio da msg */
     Tp_Msg = 7;
     L = 0;
     F = 0;
     S = 0;
     Bw = 0;
     Dest = Be;
     //Envia msg Change-root
     flag = 0;
     i = NB_VIZIN;
   }else{
     flag = 1;
   }
 }
 if(flag == 1){
   /* Preparando envio da msg */
   Tp_Msg = 1;
   L = LN;
   F = 0;
   S = 0;
   Bw = 0;
   Dest = Be;
   //Envia msg Connect(LN)
   for(i=0;i<NB_VIZIN;i++){
     if(vizinhos[2][i] == Be){
       vizinhos[0][i] = 1;
       i =  NB_VIZIN;
     }
   }
 }
}

void R_Change (void){
 P_Change(); 
}
   

void EndFila (void){
  // Verifica Fila cheia
  if(((F_Fim+1)%N_Fila) == F_Ini){
    turnOnGreenLeds();
    }else{
      int j = 0;
      for(j=0;j<=7;j++){
        Fila[F_Fim][j] = Fila[F_Ini][j];
      }
      F_Fim = (F_Fim+1)%N_Fila;
    }
}


int main(void) {
  
  initPeripherals();
  calibrateSensors();
  
 
//  int batteryPercent = 0;
  StarTest();
  Inicializa();
  Despertar();
  
int contar = 0;
int c_msg = 0;
 
 
  while(1) {
     Tp_Msg_R = Fila[F_Ini][1];

       // Estágio do algoritmo
   switch(Tp_Msg_R){
     case 1: // Resposta do Connect
       if(SN==0){
         Wakeup();
         c_msg = 0;
         int i = 0;
         for(i=0;i<=6;i++){
           if(Fila[F_Ini][i]==Fila[F_Fim][i]){
           }else{
             // Verifica Fila cheia
             if(((F_Fim+1)%N_Fila) == F_Ini){
               turnOnGreenLeds();
             }else{
               int j = 0;
               for(j=0;j<=6;j++){
                 Fila[F_Fim][j] = Fila[F_Ini][j];
               }
               F_Fim = (F_Fim+1)%N_Fila;
             }
             i = 8;
           }
         }         
         
       }else{
         if(c_msg>0){
         R_Connect();
         }
       }
       break;
       
      case 2: 
        if(Add_Vizin>NB_VIZIN){
          Add_Vizin = 0;
        }
        R_Initiate();
      break;
      
      case 3:
        if(SN==0){
         Wakeup();
         c_msg = 0;
         int i = 0;
         for(i=0;i<=6;i++){
           if(Fila[F_Ini][i]==Fila[F_Fim][i]){
           }else{
             // Verifica Fila cheia
             if(((F_Fim+1)%N_Fila) == F_Ini){
               turnOnGreenLeds();
             }else{
               int j = 0;
               for(j=0;j<=6;j++){
                 Fila[F_Fim][j] = Fila[F_Ini][j];
               }
               F_Fim = (F_Fim+1)%N_Fila;
             }
             i = 8;
           }
         }         
         
       }else{
         if(c_msg>0){
         R_Test();
         }
       }        
      break;
            
      case 4:
        R_Report();
      break;
      
      case 5:
        R_Rejected();
      break;
      
      case 6:
        R_Accept();
      break;
      
      case 7:
        R_Change();
      break;     
            
   }


   //handleRFCommands(1, 0, 0, 0, 0, 3215);
   handleRFCommands();
   

   
   // Tratativa pós envio de msg
   if(Fila[F_Ini][0] == 3){
      switch(Fila[F_Ini][1]){
        case -1:
//          if(c_msg>=0){
//            Dest = 0;
//            Tp_Msg = 0;
//          }else{
//            c_msg++;
//          }
        break;          
        
        case 1:
          c_msg++;
          // Passa para a próxima MSG da Fila[][]
            if(F_Ini == F_Fim){
              Fila[F_Ini][1] = 0;
              Fila[F_Ini][6] = 0;
            }else{
              F_Ini = (F_Ini+1)%N_Fila;
            }
        break;
    
        case 2:
          if(F_Initiate==1){
          }else{
            if(F_Initiate==0){
              // Passa para a próxima MSG da Fila[][]
              if(F_Ini == F_Fim){
                Fila[F_Ini][1] = 0;
                Fila[F_Ini][6] = 0;
              }else{
                F_Ini = (F_Ini+1)%N_Fila;
              }
            }
           }
         break;
         
         case 3:
          c_msg++;
          // Passa para a próxima MSG da Fila[][]
            if(F_Ini == F_Fim){
              Fila[F_Ini][1] = 0;
              Fila[F_Ini][6] = 0;
            }else{
              F_Ini = (F_Ini+1)%N_Fila;
            }
         break;
         
      }
   }

      
//TESTE PARA SABER SE ROBÔ TRAVOU//          
//      if(contar<=10000){
//        turnOffGreenLeds();
//      }else{
//        turnOnGreenLeds();
//      }
//      if(contar<=20000){
//        contar++;
//      }else{
//        contar = 0;
//      }
////////////////////////////////////

          
  } 
}

