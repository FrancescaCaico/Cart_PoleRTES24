// SOLUZIONE ESERCIZIO 1 
/* l'esercizio 1 richiede di implementare un nodo di controllo che interfacciandosi con i topic cart/force e /cart/pole_state mantenga il pendolo in verticale
    in particolare deve: 
        1) Leggere lo stato del pendolo inverso dal topic /cart/polestate (MESSAGGIO JOINT-STATE --> ANGOLO IN RADIANTI TRA PENDOLO E CARRELLO )
        2) Aggiornare il ciclo di controllo --> applicando una forza capace di mantenere dritto il pendolo --> REGOLATORE PID
        3) Pubblicare il valore di controllo sul topic /cart/force. Si assume nel testo che la frequenza di pubblicazione sia di 20Hz --> 50 ms
*/

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_srvs/srv/empty.hpp> // Include per il servizio di reset

using std::placeholders::_1;

// Si crea la classe che identifica il controller
class CartPoleController : public rclcpp::Node
{

public:
    CartPoleController(): Node("cartpole_controller"), kp_(20.0), ki_(5.0), kd_(10.0), setpoint_(0.0), p_angle_(0.0), previous_error_(0.0), integral_(0.0)
    {

     // Creazione del client per il servizio di reset
    reset_client_ = this->create_client<std_srvs::srv::Empty>("/reset_simulation");
        
    // Invio della richiesta di reset
    resetSimulation();

    // IL CONTROLLER DEVE: 1) Leggere lo stato del pendolo inverso dal topic /cart/polestate (MESSAGGIO JOINT-STATE --> ANGOLO IN RADIANTI TRA PENDOLO E CARRELLO )
    // QUINDI BISOGNA SOTTOSCRIVERLO AL TOPIC. 
    pole_state_sub_= this->create_subscription<sensor_msgs::msg::JointState>("/cart/pole_state", 100, std::bind(&CartPoleController::callback, this, _1));

    /* Il controller deve poi pubblicare la forza ricavata --> serve un publisher sul topic /cart/force che pubblica un messaggio fdi tipo geometry_msgs/msg/wrench */
    pole_force_publisher_=this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);

    /* Deve eseguire il ciclo di controllo del regolatore PID per tenere il pendolo dritto con una frequenza di 50 ms */
    /* La funzione wall timer chiama in modo periodico la funzione di controllo per eseguire la variazione della forza impressa.*/
    timer_controller_=this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CartPoleController::controlCart, this));

    }

private:

    void resetSimulation()
    {
        while (!reset_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "In attesa del servizio di reset...");
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future = reset_client_->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Simulazione resettata con successo");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Errore durante il reset della simulazione: %s", e.what());
        }
    }


    // La funzione callback viene eseguita in background ogni volta che c'è una nuova pubblicazione sul topic /cart/polestate
    void callback(const sensor_msgs::msg::JointState::SharedPtr state)
    {
        /* il topic /cart/polestate ritorna un messaggio JoinState: 
        header: Questo campo è di tipo std_msgs/Header e contiene informazioni standard come il timestamp e il frame di riferimento.

        name: Un array di stringhe che contiene i nomi delle giunture. Ogni nome corrisponde ai valori nei campi position, velocity e effort.

        position: Un array di numeri in virgola mobile (float64) che rappresenta la posizione attuale di ciascuna giuntura. 
        Le posizioni sono tipicamente espresse in radianti (per giunture rotazionali) o metri (per giunture prismatiche).

        velocity: Un array di numeri in virgola mobile (float64) che rappresenta la velocità attuale di ciascuna giuntura, 
        espressa in radianti al secondo (per giunture rotazionali) o metri al secondo (per giunture prismatiche).

        effort: Un array di numeri in virgola mobile (float64) che rappresenta lo sforzo (coppia o forza) applicato a ciascuna giuntura. 
        Gli sforzi sono espressi in Newton metri (Nm) per giunture rotazionali o Newton (N) per giunture prismatiche. */
        
        /* Per fare l'esercizio dato che siamo interessati all'angolo tra l'asse verticale e il pendolo ci focalizziamo sul parametro position che rappresenta l'angolo della giuntura 
        tra il pendolo e il carrello che nella visione ideale deve essere 90. Di solito è il primo parametro del messaggio Joint.*/

       // Controlliamo che ci sia un messaggio da leggere sul topic /cart/polestate
       if(state->position.empty() == false){
            //leggiamo il valore della giuntura --> in teoria ce n'è solo 1 alla posizione 0
            p_angle_=state->position[0];
            RCLCPP_INFO(this->get_logger(), "Ricavato angolo --> %f", p_angle_);
       }


    }

    void controlCart()
    {
        // Questa funzione si occupa di calcolare la forza necessaria tramite il regolatore PID e poi applicarla sul carrellino.
        double result=calculatePIDValue(p_angle_);

        // applichiamo la forza al carrellino pubblicando sul topic /cart/force un oggetto di tipo geometry wrench
        geometry_msgs::msg::Wrench wrench_mess; 
        wrench_mess.force.y=result;
        pole_force_publisher_->publish(wrench_mess);
        RCLCPP_INFO(this->get_logger(), "Pubblicata la forza %f", result);

    }

    double calculatePIDValue(double c_angle){

        // il regolatore pid lavora su tre coefficienti. Proporzionale Integrativo e Derivativo. IN base allo spasamento tra l'angolo del pendolo e quello dell'asse verticale si misura la forza da imprimere
        double errore_= setpoint_ - c_angle; 

        //provo a calcolare dt_ come intervallo di tempo tra due pubblicazioni diverse. 


        double p_coff= kp_*errore_;
        double d_coff=(errore_ - previous_error_)/ dt_;
        integral_+=errore_*dt_;

        double value= p_coff + ki_ * integral_ + kd_ * d_coff;
        previous_error_=errore_;
        RCLCPP_INFO(this->get_logger(), "Errore: %f, P: %f, I: %f, D: %f, PID: %f",errore_, p_coff, ki_ * integral_, d_coff, value);
        return value;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pole_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr  pole_force_publisher_;
    rclcpp::TimerBase::SharedPtr  timer_controller_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;

    double p_angle_;
    double kp_ ;
    double ki_;
    double kd_;
    double integral_;
    double previous_error_;
    double setpoint_;
    double pre_pub_time; 
    double curr_pub_time; 

    const double dt_ = 0.05;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartPoleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}