#include <gtk/gtk.h>
#include <vte/vte.h>
#include <iostream>
#include <string>
#include <vector>

class Terminal
{
private:
    void create_terminal(GtkWidget* container)
    {
        GtkWidget* terminal = vte_terminal_new();
        vte_terminal_set_cursor_blink_mode(VTE_TERMINAL(terminal), VTE_CURSOR_BLINK_OFF);

        const char* command = "/usr/bin/bash";
        char** commandArray = new char*[2];

        commandArray[0] = new char[strlen(command) + 1];
        strcpy(commandArray[0], command);

        commandArray[1] = nullptr;

        vte_terminal_spawn_sync(VTE_TERMINAL(terminal),
                                VTE_PTY_DEFAULT,
                                NULL,
                                commandArray,
                                NULL, G_SPAWN_DEFAULT, NULL,
                                NULL, NULL,
                                NULL, NULL
                                );
        gtk_container_add(GTK_CONTAINER(container), terminal);
        // Clean up memory
        delete[] commandArray;
        delete command;
        delete terminal;
    }
public:
    Terminal();
    ~Terminal();
};

Terminal::Terminal(/* args */)
{
}

Terminal::~Terminal()
{
}
