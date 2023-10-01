#include <gtk/gtk.h>
#include <vte/vte.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>
#include <cmath>

struct CommandItem{
    std::string window_name;
    std::string command;
};

struct JSONData
{
    std::string host;
    std::string ip;
    std::string password;
    std::vector<CommandItem> item;
    
    JSONData(std::string host, std::string ip_addr, std::string pass, CommandItem first_item):
        host(host), ip(ip_addr), password(pass), item({first_item}){}
    
    void add_item(CommandItem add_item){
        item.push_back(add_item);
    }

    int size() const
    {
        return item.size();
    }
};

struct Command{
    std::string host;
    std::string ip;
    std::string password;
    CommandItem item;
    Command(std::string host, std::string ip_address, std::string pass, CommandItem command_item):
        host(host), ip(ip_address), password(pass), item(command_item){}
};

std::vector<JSONData> read_json(std::string path)
{
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        throw;
    }

    Json::Value jsonData;
    Json::CharReaderBuilder reader;
    JSONCPP_STRING errs;
    Json::parseFromStream(reader, inputFile, &jsonData, &errs);

    std::vector<JSONData> data;
    for (const auto& item : jsonData) {
        CommandItem first_command = {
            item["commands"][0]["name"].asString(),
            item["commands"][0]["command"].asString()
        };
        JSONData data_json = JSONData(
            item["host"].asString(),
            item["ip"].asString(),
            item["pass"].asString(),
            first_command
        );
        for (int i = 1; i < item["commands"].size(); i++)
        {
            CommandItem cmd_item = {item["commands"][i]["name"].asString(), item["commands"][i]["command"].asString()};
            data_json.add_item(cmd_item);
        }
        data.push_back(data_json);
    }
    return data;
}


struct TerminalPair{
    GtkWidget* terminal;
    std::string command;
    std::string client;
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
    const GdkRGBA _green_btn = {50, 200, 90, 255};
    const GdkRGBA _red_btn = {200, 50, 50, 255};
    if(data->status <= 0)
    {
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), data->client.c_str(), -1);
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "clear", -1);
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "lsb_release -a", -1);
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

        gtk_button_set_label(button, "Disconnect");
        data->status = 1;
    }else{
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "exit", -1);
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "lsb_release -a", -1);
        vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);

        gtk_button_set_label(button, "Connect");
        data->status = 0;
    }
}

static void run_button_clicked(GtkButton* button, gpointer user_data) {
    TerminalPair* data = cast_data(user_data);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), data->command.c_str(), -1);
    vte_terminal_feed_child(VTE_TERMINAL(data->terminal), "\n", -1);
}

static GtkWidget* create_terminal(GtkWidget* container, const char *client) {
    GtkWidget* terminal = vte_terminal_new();

    vte_terminal_set_cursor_blink_mode(VTE_TERMINAL(terminal), VTE_CURSOR_BLINK_OFF);

    const char* command = "/usr/bin/bash";

    const char* homeDirectory = getenv("HOME");

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


void create_terminal_with_buttons(GtkWidget* container, const char *cmd, std::string client) {
    GtkWidget* box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

    gtk_container_add(GTK_CONTAINER(container), box);

    GtkWidget* terminal_widget = create_terminal(box, client.c_str());

    // Create a grid to hold the buttons
    GtkWidget* button_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(button_grid), 5);

    GtkWidget* prepare_button = gtk_button_new_with_label("Connect");
    GtkWidget* run_button = gtk_button_new_with_label("Run");

    TerminalPair* terminal_pair = new TerminalPair(terminal_widget, cmd, client.c_str(), -1);

    g_signal_connect(prepare_button, "clicked", G_CALLBACK(prepare_button_clicked), (gpointer)terminal_pair);
    g_signal_connect(run_button, "clicked", G_CALLBACK(run_button_clicked), (gpointer)terminal_pair);

    gtk_widget_set_halign(button_grid, GTK_ALIGN_CENTER);

    // Add buttons to the grid
    gtk_grid_attach(GTK_GRID(button_grid), prepare_button, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(button_grid), run_button, 1, 0, 1, 1);

    // Pack the grid at the bottom of the box
    gtk_box_pack_end(GTK_BOX(box), button_grid, FALSE, FALSE, 1);
}

std::string generate_command(std::string host, std::string ip, std::string pass)
{
    return "sshpass -p '" + pass + "' ssh -t " + host + "@" + ip + " 'exec $SHELL'";
}

int main(int argc, char* argv[]) {
    std::vector<JSONData> json_data;
    if(argc == 2)
    {
        std::cout << "Loading JSON file in " << argv[1] << std::endl;
        json_data = read_json(argv[1]);
    } else {
        std::cout << "Using default JSON data" << std::endl;
        json_data = read_json("../src/command.json");
    }


    gtk_init(&argc, &argv);

    GtkWidget* window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "EMIRO Autonomous Drone System");
    // gtk_window_set_default_size(GTK_WINDOW(window), 800, 600);
    gtk_window_maximize(GTK_WINDOW(window));
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget* main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), main_box);

    GtkWidget* upper_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    GtkWidget* lower_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);

    gtk_box_pack_start(GTK_BOX(main_box), upper_box, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(main_box), lower_box, TRUE, TRUE, 0);

    std::vector<Command> cmd;
    for (auto itm: json_data)
    {
        std::string host = itm.host;
        std::string ip = itm.ip;
        std::string pass = itm.password;
        for(auto d: itm.item)
            cmd.push_back({host, ip, pass, d});
    }
    
    std::cout << "Open terminal" << std::endl;
    int tot_command = cmd.size();
    if(tot_command == 0)
        std::cout << "JSON File is blank or Failed to read" << std::endl;
    else{
        int atas = 0, bawah = 0;
        if(tot_command%2 == 0){
            atas = tot_command/2;
            bawah = atas;
        }else{
            atas = std::ceil(tot_command/2.0f);
            bawah = std::floor(tot_command/2.0f);
        }

        // Row 1
        for (int i = 0; i < atas; ++i) {
            GtkWidget* terminal_frame = gtk_frame_new(NULL);
            
            gtk_frame_set_label(GTK_FRAME(terminal_frame), cmd[i].item.window_name.c_str());
            
            GtkWidget* terminal_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
            
            gtk_container_add(GTK_CONTAINER(terminal_frame), terminal_box);

            create_terminal_with_buttons(terminal_box, cmd[i].item.command.c_str(), generate_command(cmd[i].host, cmd[i].ip, cmd[i].password));
            
            gtk_box_pack_start(GTK_BOX(upper_box), terminal_frame, TRUE, TRUE, 5);
        }
        // Row 2
        for (int i = atas; i < cmd.size(); ++i) {
            GtkWidget* terminal_frame = gtk_frame_new(NULL);

            GtkWidget* terminal_frame_buttons = gtk_frame_new(NULL);
            
            gtk_frame_set_label(GTK_FRAME(terminal_frame), cmd[i].item.window_name.c_str());
            
            GtkWidget* terminal_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
            
            gtk_container_add(GTK_CONTAINER(terminal_frame), terminal_box);

            create_terminal_with_buttons(terminal_box, cmd[i].item.command.c_str(), generate_command(cmd[i].host, cmd[i].ip, cmd[i].password));
            
            gtk_box_pack_start(GTK_BOX(lower_box), terminal_frame, TRUE, TRUE, 5);
        }
    }


    gtk_widget_show_all(window);
    gtk_main();

    return 0;
}
