/* Probe so linuxdeploy-plugin-gtk collects GTK3 + WebKitGTK 4.0 from Ubuntu 22.04.
 * gtk_init alone does not pull libwebkit2gtk; pywebview needs that .so at runtime.
 */
#include <gtk/gtk.h>
#include <webkit2/webkit2.h>

int main(int argc, char **argv)
{
    gtk_init(&argc, &argv);
    (void)webkit_get_major_version();
    (void)webkit_get_minor_version();
    (void)webkit_get_micro_version();
    return 0;
}
