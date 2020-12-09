#include <stdio.h>
#include <stdlib.h>
#include <user_interface.h>
#include <espconn.h>
#include <osapi.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <mem.h>

uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


/***************************************************************************************************
                                             CONSTANTES
                                            ------------
 * seg: patrón usado para activar los temporizadores por Software. Establecido en 1000 ms/1 segundo
 * totalPorcionesTiempo: cantidad de tiempo asignada al estado de escaneo de asignación 
                    de espacios temporales por canal
 * nroPaquetesxEnvio: define el número de paquetes a enviar al servidor en cada comunicación
 * nroBytesxPaquete: define el número de bytes que requiere cada paquete de información recolectada
****************************************************************************************************/
#define totalPorcionesTiempo 13;
#define seg 1000
#define nroPaquetesxEnvio 10
#define nroBytesxPaquete 14

/***************************************************************************************************
                                     VARIABLES DE CONTROL
                                    ----------------------
 * cambioPendiente: usada para determinar en cada ciclo del temporizador por hardware
                    si debe o no debe hacerse un cambio de estado
 * reintentar: lleva la cuenta de la cantidad de veces en que se debe intentar establecer una conexión WiFi,
               así como también de la cantidad de veces que debe intentar establecer una conexión
               con el servidor TCP antes de un reinicio del dispositivo
 * estado: enumeración para controlar el paso de un estado a otro: distribuir, actualizar, enviar y leer
 * General: estructura para controlar los parámetros determinantes para realizar un cambio de estado
****************************************************************************************************/
bool cambioPendiente;

typedef enum 
{
    distribuir,
    actualizar,
    enviar,
    leer
}Estado;

Estado estado;


typedef struct{
    bool horaNoActualizada; //true, cuando haga falta hora
    bool bufferLleno;       //true, cuando buffer esté lleno
    bool esperandoServidor; //true, cuando se esté esperando respuesta del servidor
    bool canalesNoAsignados;//true, cuando canales no estén asignados
}acciones;
acciones controlGeneral;

/***************************************************************************************************
                                     VARIABLES TEMPORALES
                                    ----------------------
 * tiempoInicial: almacena el tiempo exacto en que se sincorniza la hora con el servidor, tomando
                  en cuenta el tiempo de operación desde el inicio del dispositivo
 * temporizador: variable para el funcionamiento del temporizador por software
 * _time : estructura para el control exacto del tiempo en que se recibe un conjunto de información
****************************************************************************************************/

typedef	struct	{
	uint8 year;
	uint8 mon;
    uint8 day;
    uint8 hour;
    uint8 min;
    uint8 sec;
    uint8 limiteMes;
}tm;

tm _time;


uint32 tiempoInicial;
os_timer_t temporizador;

/***************************************************************************************************
                           VARIABLES DE COMUNICACIÓN CON EL SERVIDOR
                           ------------------------------------------
 * own_espconn: descriptor para las conexiones del ESP8266, maneja status y errores de conexiones 
                TCP y UDP
 * user_tcp: estructura que contiene toda la información relativa a la conexión TCP: puertos, IP y 
             funciones de manejo de eventos TCP
****************************************************************************************************/
struct espconn own_espconn;
esp_tcp user_tcp;
/***************************************************************************************************
                        VARIABLES DE ASIGNACIÓN DE TIEMPOS DE LECTURA POR CADA CANAL
                        -------------------------------------------------------------
 * listaCanales: lista que guarda la información del espacio de tiempo de lectura asignado a cada canal
                 según el nro de paquetes recibidos por cada uno de ellos en el proceso de escaneo
 * nroCanal: controla el cambio de canal en el estado de asignación de tiempo por canales
 * nroLista: controla el canal miembro de la lista listaCanales que se debe establecer en cualquier
             momento de la lectura de datos, según el tiempo de lectura asignado al mismo
                        para el funcionamiento del temporizador por hardware
 * nroSlots: controla que cada canal se mantenga en lectura de datos de acuerdo al tiempo asignado
             a este en el proceso de escaneo
****************************************************************************************************/


typedef struct {
    uint8 timeSlot;
    uint32 nroPaquetes;
    uint8 nroCanal;
}ordenCanales;
ordenCanales listaCanales[15];

/***************************************************************************************************
                        VARIABLES DE LECTURA DE DIRECCIONES MAC - PROBE REQUEST
                        -------------------------------------------------------
 * nroCanal: controla el cambio de canal en el estado de asignación de tiempo por canales
 * nroLista: controla el canal miembro de la lista listaCanales que se debe establecer en cualquier
             momento de la lectura de datos, según el tiempo de lectura asignado al mismo
                        para el funcionamiento del temporizador por hardware
 * nroSlots: controla que cada canal se mantenga en lectura de datos de acuerdo al tiempo asignado
             a este en el proceso de escaneo
 * tamayioAcumulado: lleva la cuenta de la cantidad de paquetes capturados, así se arma el buffer de
                     envío TCP del tamaño adecuado
 * bufferdeAlmacenamiento: lista que guarda la información del espacio de tiempo de lectura asignado a cada 
                           canal según el nro de paquetes recibidos por cada uno de ellos en el proceso de escaneo
 * bufferdeEnvio: buffer que contiene la información de las direcciones MAC detectadas, ya serializada
                  y lista para enviar

****************************************************************************************************/
uint8 nroCanal=0;
uint8 nroLista=0;
uint8 nroSlots=0;

uint16 tamayioAcumulado=0;
uint8 bufferdeAlmacenamiento [nroPaquetesxEnvio][nroBytesxPaquete]={0};
uint8 bufferdeEnvio[nroPaquetesxEnvio*nroBytesxPaquete+1];

/***************************************************************************************************
                        ESTRUCTURAS PARA LECTURA DE TRAMAS 802.11
                        -------------------------------------------------------
 * RxControl: contiene información técnica precisa sobre la conexión con el STA del cual se recibe la
              trama capturada: RSSI, canal, MCS y otros.
 * sniffer_buf: almacena la data completa dentro de la trama capturada, de aquí se extra la dirección
                MAC del dispositivo que envía
****************************************************************************************************/


struct RxControl { 
    signed rssi:8;            
    unsigned rate:4; 
    unsigned is_group:1; 
    unsigned:1; 
    unsigned sig_mode:2;      
    unsigned legacy_length:12;
    unsigned damatch0:1; 
    unsigned damatch1:1; 
    unsigned bssidmatch0:1; 
    unsigned bssidmatch1:1; 
    unsigned MCS:7;           
                            
    unsigned CWB:1;
    unsigned HT_length:16;
    unsigned Smoothing:1; 
    unsigned Not_Sounding:1; 
    unsigned:1; 
    unsigned Aggregation:1; 
    unsigned STBC:2; 
    unsigned FEC_CODING:1; 
        unsigned SGI:1; 
    unsigned rxend_state:8; 
    unsigned ampdu_cnt:8; 
    unsigned channel:4; 
    unsigned:12; 
};

typedef struct { 
    struct RxControl rx_ctrl; 
    uint8_t buf[112]; 
    u16 cnt;    
    u16 len;  
} sniffer_buf;

/***************************************************************************************************
                                                FUNCIONES
                                              -------------
****************************************************************************************************/

//___________________________________________________________________________
void ICACHE_FLASH_ATTR cambiaEstado(Estado nuevoEstado)
{
    estado=nuevoEstado;
    cambioPendiente=true;
}

//___________________________________________________________________________
void actualizarHora(char *cadenaTemporal){
    
    char *variableTemporal;

    variableTemporal = strtok(cadenaTemporal,"/");
    _time.year=atoi(variableTemporal);
    os_printf("\nAño: %d", _time.year);

    variableTemporal = strtok(NULL,"/");
    _time.mon=atoi(variableTemporal);
    os_printf("\nMes: %d", _time.mon);

    variableTemporal = strtok(NULL,"/");
    _time.day=atoi(variableTemporal);
    os_printf("\nDía: %d", _time.mon);

    variableTemporal = strtok(NULL,"/");
    _time.hour=atoi(variableTemporal);
    os_printf("\nHora: %d", _time.hour);

    variableTemporal = strtok(NULL,"/");
    _time.min=atoi(variableTemporal);
    os_printf("\nMinuto: %d", _time.min);

    variableTemporal = strtok(NULL,"/");
    _time.sec=atoi(variableTemporal);
    os_printf("\nMinuto: %d", _time.sec);

    switch (_time.mon)
    {

        case 1:
            _time.limiteMes=31;
            break;
        case 2:
            _time.limiteMes=28;
            break;
        case 3:
            _time.limiteMes=31;
            break;  
        case 4:
            _time.limiteMes=30;
            break; 
        case 5:
            _time.limiteMes=31;
            break;
        case 6:
            _time.limiteMes=30;
            break; 
        case 7:
            _time.limiteMes=31;
            break;
        case 8:
            _time.limiteMes=31;
            break; 
        case 9:
            _time.limiteMes=30;
            break;
        case 10:
            _time.limiteMes=31;
            break;
        case 11:
            _time.limiteMes=30;
            break;
        case 12:
            _time.limiteMes=31;
            break;
        default:
            _time.limiteMes=30;
            break;

    }

}

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR recibirTCP(void *arg, char *pusrdata, unsigned short length)
{
    
    tiempoInicial = system_get_time();
    actualizarHora(pusrdata);
    os_printf("\nMensaje recibido: !!! %s \r\n", pusrdata);
    os_free(bufferdeEnvio);
    tamayioAcumulado=0;
    controlGeneral.bufferLleno=false;
    controlGeneral.esperandoServidor=false;
    controlGeneral.horaNoActualizada=false;

    os_printf("\nDesconectando TCP...\r\n");
    espconn_disconnect(&own_espconn);

};

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR enviadoTCP(void *arg)
{
    
    os_printf("\nDatos enviados\r\n");
 
};

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR desconectadoTCP(void *arg)
{
    
    os_printf("\nDesconexión TCP\r\n");

    if(wifi_station_disconnect())
    {
        
        os_printf("\nDesconexión WiFi\r\n");

        if(controlGeneral.canalesNoAsignados==true)
        {
            
            cambiaEstado(distribuir);

        }
        else
        {
            
            cambiaEstado(leer);

        }

    }

};

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR enviarTCP(struct espconn *pespconn)
{

    espconn_send(pespconn, bufferdeEnvio, sizeof(bufferdeEnvio));
    
}

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR conectadoTCP(void *arg)
{
    
    controlGeneral.esperandoServidor=true;
    struct espconn *pespconn = arg;

    os_printf("\nConexión con el servidor establecida\r\n");

    espconn_regist_recvcb(pespconn, recibirTCP);
    espconn_regist_sentcb(pespconn, enviadoTCP);
    espconn_regist_disconcb(pespconn, desconectadoTCP);

    espconn_set_opt(pespconn, 0x04);
    enviarTCP(pespconn);

}

//_______________________________________________________________________________________
LOCAL void ICACHE_FLASH_ATTR reconectarTCP(void *arg, sint8 err)
{
      
    os_printf("\nProblema de conexión con el servidor, código: %d\r\n",err);

}

//_______________________________________________________________________________________
void solicitudIP(void)
{
    
    os_timer_disarm(&temporizador);
    struct ip_info ipconfig;

    wifi_get_ip_info(STATION_IF, &ipconfig);
  
    if (wifi_station_get_connect_status() == STATION_GOT_IP && ipconfig.ip.addr != 0)
    {

        os_printf("\nIP obtenida\r\n");

        own_espconn.type = ESPCONN_TCP;
        own_espconn.state = ESPCONN_NONE;
        own_espconn.proto.tcp = &user_tcp;
        
        const char esp_server_ip[4] = {000, 000, 0, 000}; 
        os_memcpy(own_espconn.proto.tcp->remote_ip, esp_server_ip, 4); 
        own_espconn.proto.tcp->remote_port = 3000;
        own_espconn.proto.tcp->local_port = espconn_port();

        espconn_regist_connectcb(&own_espconn, conectadoTCP);
        espconn_regist_reconcb(&own_espconn, reconectarTCP); 
        espconn_connect(&own_espconn);
    
    } 
   else
   {
       
       if ((wifi_station_get_connect_status() == STATION_WRONG_PASSWORD ||
                wifi_station_get_connect_status() == STATION_NO_AP_FOUND ||
                wifi_station_get_connect_status() == STATION_CONNECT_FAIL)) 
                {
                    
                    os_printf("\nProblema de configuración o de indisponibilidad de AP\r\n");
                    
                } 
      else
      {
          //Si las fallas no dependen de que algo esté mal configurado o no exista el AP, se intenta de nuevo
          if(wifi_station_get_connect_status() == STATION_IDLE)
          {
              
              os_printf("\nError Station Idle");

          }
          if(wifi_station_get_connect_status() == STATION_CONNECTING)
          {
              
              os_printf("\nConectando...");

          }

              os_timer_disarm(&temporizador);
              os_timer_setfn(&temporizador,(os_timer_func_t *)solicitudIP,NULL);
              os_timer_arm(&temporizador,2000, true);
              
        }
    
    }

};
 
//_______________________________________________________________________________________
void ICACHE_FLASH_ATTR configEstacion(void)
{
    
    char ssid[32] = "****";
    char password[64] = "*************";
    struct station_config stationConf;    
    
    wifi_station_get_config(&stationConf);

    if(strcmp(stationConf.ssid, ssid)!=0)
    {
        
        os_printf("\nAsignando credenciales WiFi...");
        os_memcpy(&stationConf.ssid, ssid, 32); 
        os_memcpy(&stationConf.password, password, 64);
        wifi_station_set_config(&stationConf);
        stationConf.bssid_set = 0;

    }

    wifi_station_connect();
    
}

//_______________________________________________________________________________________
tm calcularTiempo(uint32 tiempoActual){
    
    uint32 ayio;
    uint32 mes;
    uint32 dia;
    uint32 hora;
    uint32 minutos;
    uint32 segundos;
    uint32 deltaTiempo;
    tm tiempoCorriente;

    deltaTiempo = tiempoActual-tiempoInicial;

    uint32 sumaDia=0;
    uint32 sumaMes=0;
    uint32 sumaAyio=0;

    uint32 sumaHora = deltaTiempo/3600000000;
    uint32 restoHora = deltaTiempo%3600000000;


    uint32 sumaMinuto = restoHora/60000000;
    uint32 restoMinuto = restoHora%60000000;
    
    uint32 sumaSegundo = restoMinuto/1000000;
    uint32 restoSegundo = restoMinuto%1000000;

    if(_time.sec+sumaSegundo>=60)
    {
        
        sumaMinuto++;
        sumaSegundo=sumaSegundo-60;

    }
    if(_time.min+sumaMinuto>=60)
    {

        sumaHora++;
        sumaMinuto=sumaMinuto-60;

    }
    if(_time.hour+sumaHora>=24)
    {
        
        sumaDia++;
        sumaHora=sumaHora-24;

    }
    if(_time.day+sumaDia>_time.limiteMes)
    {
        
        sumaMes++;
        sumaDia=sumaDia-_time.limiteMes;

    }
    if(_time.mon+sumaMes>12)
    {
        
        sumaAyio++;
        sumaMes=sumaMes-12;

    }

    tiempoCorriente.year=_time.year+sumaAyio;
    tiempoCorriente.mon=_time.mon+sumaMes;
    tiempoCorriente.day=_time.day+sumaDia;
    tiempoCorriente.hour=_time.hour+sumaHora;
    tiempoCorriente.min=_time.min+sumaMinuto;
    tiempoCorriente.sec=_time.sec+sumaSegundo;

    return tiempoCorriente;

}

//_______________________________________________________________________________________
void almacenarDatos(sniffer_buf *paquete, tm momento){
    
    bufferdeAlmacenamiento[tamayioAcumulado][0] = momento.year;
    bufferdeAlmacenamiento[tamayioAcumulado][1] = momento.mon; 
    bufferdeAlmacenamiento[tamayioAcumulado][2] = momento.day; 
    bufferdeAlmacenamiento[tamayioAcumulado][3] = momento.hour; 
    bufferdeAlmacenamiento[tamayioAcumulado][4] = momento.min; 
    bufferdeAlmacenamiento[tamayioAcumulado][5] = momento.sec;

    bufferdeAlmacenamiento[tamayioAcumulado][6] = paquete->buf[10];
    bufferdeAlmacenamiento[tamayioAcumulado][7] = paquete->buf[11]; 
    bufferdeAlmacenamiento[tamayioAcumulado][8] = paquete->buf[12]; 
    bufferdeAlmacenamiento[tamayioAcumulado][9] = paquete->buf[13]; 
    bufferdeAlmacenamiento[tamayioAcumulado][10] = paquete->buf[14]; 
    bufferdeAlmacenamiento[tamayioAcumulado][11] = paquete->buf[15];

    bufferdeAlmacenamiento[tamayioAcumulado][12] = paquete->rx_ctrl.channel;
    bufferdeAlmacenamiento[tamayioAcumulado][13] = abs(paquete->rx_ctrl.rssi);

    tamayioAcumulado++;
    paquete=NULL;

    os_printf("Se guardó un dato\n");

}


///__________________________________________________________________________________
void ordenarLista(ordenCanales* listaAOrdenar){
    
    uint8 x, y, max;
    ordenCanales tmp;

    for( x=0; x<14; x++ )
    {
        max=x;
        for( y=x+1; y<14; y++ )
        {
            
            if( listaAOrdenar[max].nroPaquetes<listaAOrdenar[y].nroPaquetes )
            {
                
                max=y;
                
            }

        }
        tmp=listaAOrdenar[max];
        listaAOrdenar[max]=listaAOrdenar[x];
        listaAOrdenar[x]=tmp;

    }

}

///__________________________________________________________________________________
void definirPrioridad( ordenCanales *lista, uint32 totalPaquetes ){
    
    uint8 i, quedaTiempo=totalPorcionesTiempo;
    double porcentaje, porcentajeTotal = 0;
    
    for( i=0; i<13; i++ )
    {
        
        porcentaje=(double)lista[i].nroPaquetes/(double)totalPaquetes;
        lista[i].timeSlot=porcentaje*totalPorcionesTiempo;
        porcentajeTotal=porcentajeTotal+porcentaje;
        quedaTiempo-=lista[i].timeSlot;
        
        if( porcentajeTotal >= 0.95 )
        {
            
            break;

        }
    }
    lista[0].timeSlot+=quedaTiempo;

}


///__________________________________________________________________________________
bool esManagement(sniffer_buf *paquete){


    if  ( (paquete->buf[0] & 0b00110000) == 0b00000000 ) 
    {
        
        return true;

    }
    else {return false;} 

}

//_______________________________________________________________________________________
bool esProbeRequest(sniffer_buf *paquete){ 

    if  ( (paquete->buf[0] & 0b00110000) == 0b00000000)
    {
        
        if ( ( paquete->buf[0] & 0b00001111 ) == 0b00000100) 
        {
            
            return true;
        }
        else {return false;}
    }
    else {return false;} 

}

//_______________________________________________________________________________________
void Captura(uint8_t *buffer, uint16_t len_buf){
    
    ets_intr_lock();

    uint32 tiempoActual = system_get_time();
    sniffer_buf *paquete =  (sniffer_buf*) buffer;

    if(tamayioAcumulado==nroPaquetesxEnvio)
    {
        
        controlGeneral.bufferLleno=true;
        wifi_promiscuous_enable(0);
        paquete=NULL;
        return;

    }
    if(controlGeneral.horaNoActualizada==true)
    {

        wifi_promiscuous_enable(0);
        paquete=NULL;
        cambiaEstado(enviar);
        return;

    }
    if ( esManagement(paquete)  == true )
    {
        
        almacenarDatos(paquete, calcularTiempo(tiempoActual));
        
    }

    buffer=NULL;
    ets_intr_unlock();

}

//_______________________________________________________________________________________
void cambiarCanal(){

    if(controlGeneral.bufferLleno==true)
    {
        
        os_printf("\nBuffer lleno");
        os_timer_disarm(&temporizador);
        cambiaEstado(enviar);
        return;

    }
    
    if(listaCanales[nroLista].timeSlot==0)
    {
        
        nroLista=0;

    }

    wifi_set_channel(listaCanales[nroLista].nroCanal);
    nroSlots++;
    
    if(nroSlots==listaCanales[nroLista].timeSlot)
    {
        
        nroLista++;
        nroSlots=0;

    }
  
}

//_______________________________________________________________________________________
void iniciarLectura(){
    
    wifi_station_disconnect();
    wifi_promiscuous_enable(0);
    wifi_promiscuous_enable(1);
    wifi_set_promiscuous_rx_cb(Captura);

    os_timer_disarm(&temporizador);
    os_timer_setfn(&temporizador, (os_timer_func_t *) cambiarCanal, NULL);
    os_timer_arm(&temporizador, seg, true); 

}

///__________________________________________________________________________________
void escaneoCanales(){
    
    nroCanal++;

    if(nroCanal==14)
    {

        wifi_promiscuous_enable(0);
        os_timer_disarm(&temporizador);
        
        uint32 totalPaquetes=0;    

        for(uint8 i=1;i<14;i++)
        {
            
            totalPaquetes=totalPaquetes+listaCanales[i].nroPaquetes;
       
        }

        ordenarLista(listaCanales);
        definirPrioridad(listaCanales, totalPaquetes);

        for(uint8 i=0;i<14;i++)
        { 

            os_printf("Nro de paquetes de canal %d:\n __%d__\n", listaCanales[i].nroCanal, listaCanales[i].nroPaquetes);
            os_printf("Tiempo asignado: %d", listaCanales[i].timeSlot);
            
        }
        nroCanal=0;

        controlGeneral.canalesNoAsignados=false;
        cambiaEstado(leer);
     
    }
    else
    {

        os_printf("\nEscaneando canal: %d", nroCanal);
        wifi_set_channel(nroCanal);
    
    }
}

///__________________________________________________________________________________
void densidadPorCanal(uint8_t *buffer, uint16_t len_buf){

    sniffer_buf *paquete = (sniffer_buf*) buffer;

    if ( esManagement(paquete)  == true )
    {

        ETS_INTR_LOCK();
        listaCanales[paquete->rx_ctrl.channel].nroPaquetes++;
        listaCanales[paquete->rx_ctrl.channel].nroCanal=paquete->rx_ctrl.channel;
        ETS_INTR_UNLOCK();

    }      

}

///__________________________________________________________________________________
void ICACHE_FLASH_ATTR distribuirTiempo(){
    
    os_timer_disarm(&temporizador);
    os_timer_setfn(&temporizador, (os_timer_func_t *)escaneoCanales, NULL);
    os_timer_arm(&temporizador, seg, 1);

    wifi_promiscuous_enable(0);
    wifi_set_promiscuous_rx_cb(densidadPorCanal);
    wifi_promiscuous_enable(1);

}

///__________________________________________________________________________________
void ICACHE_FLASH_ATTR inicializarEstados(){
   
    controlGeneral.bufferLleno=false;
    controlGeneral.canalesNoAsignados=true;
    controlGeneral.esperandoServidor=false;
    controlGeneral.horaNoActualizada=true;
    cambioPendiente=true;
    estado=actualizar;

}

///__________________________________________________________________________________
void ICACHE_FLASH_ATTR actualizarCliente(void){

    cambioPendiente=false;    
    configEstacion();
    bufferdeEnvio[0]=1;

    os_timer_disarm(&temporizador);
    os_timer_setfn(&temporizador, (os_timer_func_t *)solicitudIP, NULL);
    os_timer_arm(&temporizador, 100, 0);

}

//_______________________________________________________________________________________
void prepararEnvio(){
    
    uint16 ubicacion=0;
    bufferdeEnvio[0]=tamayioAcumulado;
    bufferdeEnvio[1]= bufferdeAlmacenamiento[1][0];
    bufferdeEnvio[2]= bufferdeAlmacenamiento[1][1];
    bufferdeEnvio[3]= bufferdeAlmacenamiento[1][2];
    bufferdeEnvio[4]= bufferdeAlmacenamiento[1][3];
    bufferdeEnvio[5]= bufferdeAlmacenamiento[1][4];
    bufferdeEnvio[6]= bufferdeAlmacenamiento[1][5];
    bufferdeEnvio[7]= bufferdeAlmacenamiento[1][6];
    bufferdeEnvio[8]= bufferdeAlmacenamiento[1][7];
    bufferdeEnvio[9]= bufferdeAlmacenamiento[1][8];
    bufferdeEnvio[10]= bufferdeAlmacenamiento[1][9];
    bufferdeEnvio[11]= bufferdeAlmacenamiento[1][10];
    bufferdeEnvio[12]= bufferdeAlmacenamiento[1][11];
    bufferdeEnvio[13]= bufferdeAlmacenamiento[1][12];
    bufferdeEnvio[14]= bufferdeAlmacenamiento[1][13];

    for(int i=1; i<=tamayioAcumulado; i++)
    {

        os_printf("Recorriendo arreglo...\n\n");
        ubicacion=i*nroBytesxPaquete+1;
        bufferdeEnvio[ubicacion]= bufferdeAlmacenamiento[i][0];
        bufferdeEnvio[ubicacion+1]= bufferdeAlmacenamiento[i][1];
        bufferdeEnvio[ubicacion+2]= bufferdeAlmacenamiento[i][2];
        bufferdeEnvio[ubicacion+3]= bufferdeAlmacenamiento[i][3];
        bufferdeEnvio[ubicacion+4]= bufferdeAlmacenamiento[i][4];
        bufferdeEnvio[ubicacion+5]= bufferdeAlmacenamiento[i][5];
        bufferdeEnvio[ubicacion+6]= bufferdeAlmacenamiento[i][6];
        bufferdeEnvio[ubicacion+7]= bufferdeAlmacenamiento[i][7];
        bufferdeEnvio[ubicacion+8]= bufferdeAlmacenamiento[i][8];
        bufferdeEnvio[ubicacion+9]= bufferdeAlmacenamiento[i][9];
        bufferdeEnvio[ubicacion+10]= bufferdeAlmacenamiento[i][10];
        bufferdeEnvio[ubicacion+11]= bufferdeAlmacenamiento[i][11];
        bufferdeEnvio[ubicacion+12]= bufferdeAlmacenamiento[i][12];
        bufferdeEnvio[ubicacion+13]= bufferdeAlmacenamiento[i][13];

   }
  
}

//_______________________________________________________________________________________
void enviarDatos(){

    prepararEnvio();
    configEstacion();

    os_timer_disarm(&temporizador);
    os_timer_setfn(&temporizador, (os_timer_func_t *)solicitudIP, NULL);
    os_timer_arm(&temporizador, 100, 0);

}

//_______________________________________________________________________________________
void actualizarEstados(){

    if(cambioPendiente==true)
    {

        switch(estado)
        {

            case distribuir:
                cambioPendiente=false;
                distribuirTiempo();
                break;
            case actualizar:
                cambioPendiente=false;
                actualizarCliente();
                break;
            case leer:
                cambioPendiente=false;
                iniciarLectura(); break;
            case enviar:
                cambioPendiente=false;
                enviarDatos();
                break;

            }

    }

}

///__________________________________________________________________________________
void user_init(void){

    uart_init(115200, 115200);
    wifi_station_set_reconnect_policy(false);
    wifi_set_opmode(STATION_MODE);
    inicializarEstados();
    hw_timer_init();
    hw_timer_set_func(actualizarEstados);
    hw_timer_arm(100);

}