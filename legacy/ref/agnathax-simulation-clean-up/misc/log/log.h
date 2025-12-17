#ifndef __LOG_H
#define __LOG_H

#define LOG_MAX_MONITORS  100
#define LOG_MONITOR_ERROR -1
#define LOG_BUFFER_SIZE   81920

typedef int MonitorId;

typedef struct _Monitors Monitors;

// #ifdef __cplusplus
// extern "C" {
// #endif
	void log_filename_printf (char const *filename, char const *format, ...);
	void log_double_array (char const *filename, void *array, int n_rows, int n_cols);
	void log_printf (Monitors *monitors, MonitorId id, char const *format, ...);
	Monitors * log_monitors_new (void);
	MonitorId log_monitor_doubles (Monitors *monitors, char const *filename, int n_vars, ...);
	MonitorId log_monitor_double_array (Monitors *monitors, char const *filename, int n_vars, double *array);
	MonitorId log_monitor_floats (Monitors *monitors, char const *filename, int n_vars, ...);
	MonitorId log_monitor_float_array (Monitors *monitors, char const *filename, int n_vars, float *array);
	void log_monitors_update (Monitors *monitors);
	void log_monitors_flush (Monitors *monitors);
	void log_monitors_finish (Monitors *monitors);
	void log_monitors_destroy (Monitors *monitors);
// #ifdef __cplusplus
// }
// #endif

#endif
