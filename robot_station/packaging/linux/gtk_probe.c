/* Tiny GTK3 probe so linuxdeploy-plugin-gtk collects WebKit/GTK from Ubuntu 22.04. */
#include <gtk/gtk.h>

int main(int argc, char **argv)
{
    gtk_init(&argc, &argv);
    return 0;
}
