#include <gtk/gtk.h>
#include <vte/vte.h>
#include <iostream>

static void create_terminal(GtkWidget* container) {
    GtkWidget* terminal = vte_terminal_new();
    vte_terminal_set_cursor_blink_mode(VTE_TERMINAL(terminal), VTE_CURSOR_BLINK_OFF);

    const char* command = "/bin/bash";
    char** commandArray = new char*[2];

    commandArray[0] = new char[strlen(command) + 1];
    strcpy(commandArray[0], command);

    commandArray[1] = nullptr;

    vte_terminal_spawn_sync(VTE_TERMINAL(terminal),
                            VTE_PTY_DEFAULT,
                            NULL,           // working directory
                            commandArray, // command to run
                            NULL, GSpawnFlags::G_SPAWN_DEFAULT, NULL, // environment variables
                            NULL, NULL,     // spawn flags
                            NULL, NULL      // child setup functions
                            );
    gtk_container_add(GTK_CONTAINER(container), terminal);
    // Clean up memory
    delete[] commandArray[0];
    delete[] commandArray;
    std::cout << "Cek" << std::endl;
}

int main(int argc, char* argv[]) {
    gtk_init(&argc, &argv);

    GtkWidget* window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), 800, 600);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget* main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), main_box);

    GtkWidget* terminal_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_box_pack_start(GTK_BOX(main_box), terminal_box, TRUE, TRUE, 0);

    for (int i = 0; i < 4; ++i) {
        GtkWidget* terminal_frame = gtk_frame_new(NULL);
        gtk_frame_set_label(GTK_FRAME(terminal_frame), ("Terminal " + std::to_string(i + 1)).c_str());
        gtk_box_pack_start(GTK_BOX(terminal_box), terminal_frame, TRUE, TRUE, 5);
        create_terminal(terminal_frame);
    }

    gtk_widget_show_all(window);
    gtk_main();

    return 0;
}
