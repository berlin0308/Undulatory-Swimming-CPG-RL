#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "log.h"

typedef enum {
	LOG_TYPE_DOUBLES,
	LOG_TYPE_FLOATS,
	LOG_TYPE_INTS,
	N_LOG_TYPES
} LogType;

typedef struct {
	FILE *file;
	int n_vars;
	LogType type;
	void **variables;
	char *buffer;
} Monitor;

struct _Monitors {
	int n_monitors;
	Monitor monitors[LOG_MAX_MONITORS];
};

Monitors *
log_monitors_new ()
{
	Monitors *monitors = (Monitors*) calloc (1, sizeof (Monitors));
	return monitors;
}

void
log_monitors_destroy (Monitors *monitors)
{
	log_monitors_finish (monitors);
	free (monitors);
}

void
log_filename_printf (char const *filename,
                     char const *format,
                     ...)
{
	va_list ap;
	FILE *file;

	file = fopen (filename, "a");
	if (!file)
	{
		fprintf (stderr, "Cannot append to file \"%s\".", filename);
		return;
	}

	va_start (ap, format);
	vfprintf (file, format, ap);
	va_end (ap);

	fclose (file);
}

void
log_monitor_printf (Monitors   *monitors,
                    MonitorId   id,
                    char const *format,
                    ...)
{
	va_list ap;

	if (id >= 0 && id < monitors->n_monitors)
	{
		va_start (ap, format);
		vfprintf (monitors->monitors[id].file, format, ap);
		va_end (ap);
	}
	else
	{
		fprintf (stderr, "Monitor id out of bound: %d.\n", id);
	}
}

static MonitorId
log_monitor_new (Monitors   *monitors,
                 char const *filename,
                 int         n_vars,
                 LogType     type)
{
	/* GFile *file = g_file_new_for_path (filename); */
	/* GFileOutputStream *stream = g_file_replace (file, NULL, TRUE, G_FILE_CREATE_NONE, NULL, NULL); */
	/* GOutputStream *buffer = g_buffered_output_stream_new_sized (stream, 1000000); */
	/* g_buffered_output_stream_set_auto_grow (buffer, TRUE); */

	if (monitors->n_monitors >= LOG_MAX_MONITORS)
	{
		fprintf (stderr, "Cannot allocate more than %d monitors.\n", LOG_MAX_MONITORS);
		return LOG_MONITOR_ERROR;
	}
		
	monitors->monitors[monitors->n_monitors].file = fopen (filename, "w");

	if (!monitors->monitors[monitors->n_monitors].file)
	{
		fprintf (stderr, "Cannot write to file \"%s\".", filename);
		return LOG_MONITOR_ERROR;
	}

	monitors->monitors[monitors->n_monitors].buffer = (char *) malloc (LOG_BUFFER_SIZE);
	setvbuf (monitors->monitors[monitors->n_monitors].file, monitors->monitors[monitors->n_monitors].buffer, _IOFBF, LOG_BUFFER_SIZE);

	monitors->monitors[monitors->n_monitors].n_vars = n_vars;
	monitors->monitors[monitors->n_monitors].variables = (void**) calloc (n_vars, sizeof (void *));
	monitors->monitors[monitors->n_monitors].type = type;

	++monitors->n_monitors;

	return monitors->n_monitors - 1;
}

void
log_double_array (char const *filename,
                  void       *array,
                  int         n_rows,
                  int         n_cols)
{
	FILE *file;
	int i, j;
	
	file = fopen (filename, "w");
	
	if (!file)
	{
		fprintf (stderr, "Cannot write to file \"%s\".", filename);
		return;
	}

	for (i = 0; i < n_rows; ++i)
	{
		for (j = 0; j < n_cols; ++j)
		{
			fprintf (file, "%f ", ((double *) array)[i * n_cols + j]);
		}
		fprintf (file, "\n");
	}

	fclose (file);
}

MonitorId
log_monitor_floats (Monitors   *monitors,
                    char const *filename,
                    int         n_vars,
                    ...)
{
	MonitorId id = log_monitor_new (monitors, filename, n_vars, LOG_TYPE_FLOATS);

	if (id == LOG_MONITOR_ERROR)
	{
		return LOG_MONITOR_ERROR;
	}
	
	va_list arg_pointer;
	va_start (arg_pointer, n_vars);

	int i;
	for (i = 0; i < n_vars; ++i)
	{
		monitors->monitors[id].variables[i] = va_arg (arg_pointer, float *);
	}

	va_end (arg_pointer);

	return id;
}

MonitorId
log_monitor_doubles (Monitors   *monitors,
                     char const *filename,
                     int         n_vars,
                     ...)
{
	MonitorId id = log_monitor_new (monitors, filename, n_vars, LOG_TYPE_DOUBLES);

	if (id == LOG_MONITOR_ERROR)
	{
		return LOG_MONITOR_ERROR;
	}
	
	va_list arg_pointer;
	va_start (arg_pointer, n_vars);

	int i;
	for (i = 0; i < n_vars; ++i)
	{
		monitors->monitors[id].variables[i] = va_arg (arg_pointer, double *);
	}

	va_end (arg_pointer);

	return id;
}

MonitorId
log_monitor_float_array (Monitors   *monitors,
                         char const *filename,
                         int         n_vars,
                         float      *array)
{
	MonitorId id = log_monitor_new (monitors, filename, n_vars, LOG_TYPE_FLOATS);

	if (id == LOG_MONITOR_ERROR)
	{
		return LOG_MONITOR_ERROR;
	}

	int i;
	for (i = 0; i < n_vars; ++i)
	{
		monitors->monitors[id].variables[i] = &array[i];
	}

	return id;
}
MonitorId
log_monitor_double_array (Monitors   *monitors,
                          char const *filename,
                          int         n_vars,
                          double     *array)
{
	MonitorId id = log_monitor_new (monitors, filename, n_vars, LOG_TYPE_DOUBLES);

	if (id == LOG_MONITOR_ERROR)
	{
		return LOG_MONITOR_ERROR;
	}

	int i;
	for (i = 0; i < n_vars; ++i)
	{
		monitors->monitors[id].variables[i] = &array[i];
	}

	return id;
}

void
log_monitors_update (Monitors *monitors)
{
	int i;

	for (i = 0; i < monitors->n_monitors; ++i)
	{
		int j;

		for (j = 0; j < monitors->monitors[i].n_vars; ++j)
		{
			switch (monitors->monitors[i].type)
			{
			case LOG_TYPE_DOUBLES:
				fprintf (monitors->monitors[i].file, "%f ", * (double *) monitors->monitors[i].variables[j]);
				break;
			case LOG_TYPE_FLOATS:
				fprintf (monitors->monitors[i].file, "%f ", * (float *) monitors->monitors[i].variables[j]);
				break;
			case LOG_TYPE_INTS:
				fprintf (monitors->monitors[i].file, "%d ", * (int *) monitors->monitors[i].variables[j]);
				break;
			default:
				break;
			}
		}
		fprintf (monitors->monitors[i].file, "\n");
	}
}

void
log_monitors_flush (Monitors *monitors)
{
	int i;

	for (i = 0; i < monitors->n_monitors; ++i)
	{
		fflush (monitors->monitors[i].file);
	}
}

void
log_monitors_finish (Monitors *monitors)
{
	int i;

	for (i = 0; i < monitors->n_monitors; ++i)
	{
		monitors->monitors[i].n_vars = 0;

		fclose (monitors->monitors[i].file);
		monitors->monitors[i].file = NULL;

		free (monitors->monitors[i].buffer);
		monitors->monitors[i].buffer = NULL;

		free (monitors->monitors[i].variables);
		monitors->monitors[i].variables = NULL;
	}

	monitors->n_monitors = 0;
}
