#include <gtk/gtk.h>
#include <vte/vte.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

struct TerminalPair{
    GtkWidget* terminal;
    const char* command;
    const char* client;
    int status;
    TerminalPair(GtkWidget* term, const char *cmd, const char *cl, int stat): terminal(term), command(cmd), client(cl), status(stat){}
};

std::string get_password() {
    std::string password;
    termios oldt, newt;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    std::cout << "Enter client password: ";
    std::cin >> password;
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    std::cout << std::endl;
    
    return password;
}

TerminalPair* cast_data(gpointer data)
{
    return static_cast<TerminalPair*>(data);
}

static void prepare_button_clicked(GtkButton* button, gpointer user_data) {
    TerminalPair* data = cast_data(user_data);

    // Feed the command to the terminal
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), data->client, -1);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "clear", -1);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);
}

static void run_button_clicked(GtkButton* button, gpointer user_data) {
    TerminalPair* data = cast_data(user_data);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), data->command, -1);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);
}

static GtkWidget* create_terminal(GtkWidget* container, const char *client) {
    GtkWidget* terminal = vte_terminal_new();
    vte_terminal_set_cursor_blink_mode(VTE_TERMINAL(terminal), VTE_CURSOR_BLINK_OFF);

    const char* command = "/usr/bin/bash";

    const char* homeDirectory = getenv("HOME");

    // Construct the command array
    char** commandArray = new char*[2];
    commandArray[0] = new char[strlen(command) + 1];
    strcpy(commandArray[0], command);

    commandArray[1] = nullptr;

    vte_terminal_spawn_sync(VTE_TERMINAL(terminal),
                            VTE_PTY_DEFAULT,
                            homeDirectory,
                            commandArray,
                            NULL, G_SPAWN_DEFAULT, NULL,
                            NULL, NULL,
                            NULL, NULL
                            );

    gtk_container_add(GTK_CONTAINER(container), terminal);

    // Clean up memory
    for (int i = 0; commandArray[i] != nullptr; ++i) {
        delete[] commandArray[i];
    }
    delete[] commandArray;

    return terminal;
}


static void create_terminal_with_buttons(GtkWidget* container, const char *cmd, const char *client) {
    GtkWidget* box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(container), box);

    GtkWidget* terminal_widget = create_terminal(box, client);

    // Create a grid to hold the buttons
    GtkWidget* button_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(button_grid), 5);

    GtkWidget* prepare_button = gtk_button_new_with_label("Connect");
    GtkWidget* run_button = gtk_button_new_with_label("Run");

    TerminalPair* terminal_pair = new TerminalPair(terminal_widget, cmd, client, -1);

    g_signal_connect(prepare_button, "clicked", G_CALLBACK(prepare_button_clicked), (gpointer)terminal_pair);
    g_signal_connect(run_button, "clicked", G_CALLBACK(run_button_clicked), (gpointer)terminal_pair);

    // Add buttons to the grid
    gtk_grid_attach(GTK_GRID(button_grid), prepare_button, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(button_grid), run_button, 1, 0, 1, 1);

    // Pack the grid at the bottom of the box
    gtk_box_pack_end(GTK_BOX(box), button_grid, FALSE, FALSE, 5);
}

int main(int argc, char* argv[]) {
    std::string command[8] = {
        "./rs.sh",
        "./apm.sh",
        "roslaunch rplidar_ros rplidar_s1.launch",
        "./rs2.sh",
        "date",
        "ls /dev/video*",
        "rosservice call /mavros/set_stream_rate 0 100 1",
        "rosrun emiro 1 0 1"
    };

    if(argc < 2)
    {
        std::cout << "Use command ./<program> <client_addr>" << std::endl;
        return -1;
    }
    
    std::string client_addr = std::string(argv[1]);
    std::string password = get_password();

    std::string client_args = "sshpass -p '" + password + "' ssh -t " + client_addr + " 'exec $SHELL'";


    gtk_init(&argc, &argv);

    GtkWidget* window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "EMIRO Copter System");
    // gtk_window_set_default_size(GTK_WINDOW(window), 800, 600);
    gtk_window_maximize(GTK_WINDOW(window));
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget* main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), main_box);

    GtkWidget* upper_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    GtkWidget* lower_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);

    gtk_box_pack_start(GTK_BOX(main_box), upper_box, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(main_box), lower_box, TRUE, TRUE, 0);

    for (int i = 0; i < 4; ++i) {
        GtkWidget* terminal_frame = gtk_frame_new(NULL);
        
        gtk_frame_set_label(GTK_FRAME(terminal_frame), ("Terminal " + std::to_string(i + 1)).c_str());
        
        GtkWidget* terminal_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
        
        gtk_container_add(GTK_CONTAINER(terminal_frame), terminal_box);
        
        create_terminal_with_buttons(terminal_box, command[i].c_str(), client_args.c_str());
        
        gtk_box_pack_start(GTK_BOX(upper_box), terminal_frame, TRUE, TRUE, 5);
    }

    for (int i = 4; i < 8; ++i) {
        GtkWidget* terminal_frame = gtk_frame_new(NULL);
        GtkWidget* terminal_frame_buttons = gtk_frame_new(NULL);
        
        gtk_frame_set_label(GTK_FRAME(terminal_frame), ("Terminal " + std::to_string(i + 5)).c_str());
        
        GtkWidget* terminal_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
        
        gtk_container_add(GTK_CONTAINER(terminal_frame), terminal_box);
        
        create_terminal_with_buttons(terminal_box, command[i].c_str(), client_args.c_str());
        
        gtk_box_pack_start(GTK_BOX(lower_box), terminal_frame, TRUE, TRUE, 5);
    }

    gtk_widget_show_all(window);
    gtk_main();

    return 0;
}
