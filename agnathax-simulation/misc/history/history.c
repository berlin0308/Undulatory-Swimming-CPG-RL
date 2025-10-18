#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "history.h"

typedef double (*HistorySummaryFunc) (Histories *histories, int history_index);
typedef void (*HistoryPrintFunc) (FILE *file, Histories *histories, int history_index, int row);

typedef enum {
	TYPE_DOUBLE,
	TYPE_INT,
	N_TYPES
} HistoryType;

static int const type_size[N_TYPES] = {sizeof (double),
                                       sizeof (int)};

typedef struct {
	void *values;
	HistoryType type;
	HistoryCursor cursor;
} History;

struct _Histories
{
	History *history;
	int n_histories;
};



Histories *
histories_new_generic (int         n_variables,
                       int         max_n_rows,
                       HistoryType type)
{
	Histories *histories = (Histories*) malloc (sizeof (Histories));
	histories->n_histories = n_variables;
	histories->history = (History*) calloc (n_variables, sizeof (History));

	int i;
	for (i = 0; i < n_variables; ++i)
	{
		histories->history[i].values = (Histories*) calloc (max_n_rows, type_size[type]);
		histories->history[i].type = type;
		histories->history[i].cursor.n_values = 0;
		histories->history[i].cursor.cursor = -1;
		histories->history[i].cursor.max_n = max_n_rows;
	}

	return histories;
}

Histories *
histories_new (int n_variables,
               int max_n_rows)
{
	return histories_new_generic (n_variables, max_n_rows, TYPE_DOUBLE);
}

Histories *
histories_new_int (int n_variables,
                   int max_n_rows)
{
	return histories_new_generic (n_variables, max_n_rows, TYPE_INT);
}

void
histories_delete (Histories *histories)
{
	int i;

	if (!histories)
	{
		return;
	}

	for (i = 0; i < histories->n_histories; ++i)
	{
		if (histories->history[i].values)
		{
			free (histories->history[i].values);
		}
	}

	if (histories->history)
	{
		free (histories->history);
	}

	free (histories);
}

void
histories_add (Histories *histories,
               double    *values)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return;
	}

	int i = 0;

	for (i = 0; i < histories->n_histories; ++i)
	{
		history_add (histories, i, values[i]);
	}
}

void
histories_addf (Histories *histories,
                float     *values)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return;
	}

	int i = 0;

	for (i = 0; i < histories->n_histories; ++i)
	{
		history_add (histories, i, values[i]);
	}
}

void
histories_add_int (Histories *histories,
                   int       *values)
{
	if (histories->history[0].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return;
	}

	int i = 0;

	for (i = 0; i < histories->n_histories; ++i)
	{
		history_add_int (histories, i, values[i]);
	}
}

void
history_add (Histories *histories,
             int        history_index,
             double     value)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return;
	}

	History *history = &histories->history[history_index];
	int cursor = history_cursor_add (&history->cursor);
	
	((double *) history->values)[cursor] = value;
}

void
history_add_int (Histories *histories,
                 int        history_index,
                 int        value)
{
	if (histories->history[history_index].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return;
	}

	History *history = &histories->history[history_index];
	int cursor = history_cursor_add (&history->cursor);
	
	((int *) history->values)[cursor] = value;
}

int
history_get_length (Histories *histories,
                    int        history_index)
{
	return histories->history[history_index].cursor.n_values;
}

double
history_get_last (Histories *histories,
                  int        history_index)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (history->cursor.n_values == 0)
	{
		fprintf (stderr, "%s: no value\n", __func__);
		return 0;
	}
	else
	{
		return ((double *) history->values)[history->cursor.cursor];
	}
}

int
history_get_last_int (Histories *histories,
                      int        history_index)
{
	if (histories->history[history_index].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (history->cursor.n_values == 0)
	{
		fprintf (stderr, "%s: no value\n", __func__);
		return 0;
	}
	else
	{
		return ((int *) history->values)[history->cursor.cursor];
	}
}

double
history_get_nth_last (Histories *histories,
                      int        history_index,
                      int        nth)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (nth >= history->cursor.n_values || nth >= history->cursor.max_n)
	{
		fprintf (stderr, "%s: %d is out of bounds\n", __func__, nth);
		return 0;
	}

	int index = history_cursor_nth_last (&history->cursor, nth);

	return ((double *) history->values)[index];
}

int
history_get_nth_last_int (Histories *histories,
                          int        history_index,
                          int        nth)
{
	if (histories->history[history_index].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (nth >= history->cursor.n_values || nth >= history->cursor.max_n)
	{
		fprintf (stderr, "%s: %d is out of bounds\n", __func__, nth);
		return 0;
	}

	int index = history_cursor_nth_last (&history->cursor, nth);

	return ((int *) history->values)[index];
}

double
history_get_nth (Histories *histories,
                 int        history_index,
                 int        nth)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (nth >= history->cursor.n_values || nth >= history->cursor.max_n)
	{
		fprintf (stderr, "%s: %d is out of bounds\n", __func__, nth);
		return 0;
	}

	return ((double *) history->values)[history_cursor_nth (&history->cursor, nth)];
}

int
history_get_nth_int (Histories *histories,
                     int        history_index,
                     int        nth)
{
	if (histories->history[history_index].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];

	if (nth >= history->cursor.n_values || nth >= history->cursor.max_n)
	{
		fprintf (stderr, "%s: %d is out of bounds\n", __func__, nth);
		return 0;
	}

	return ((int *) history->values)[history_cursor_nth (&history->cursor, nth)];
}

int
histories_get_min_length (Histories *histories)
{
	int min_length = histories->history[0].cursor.n_values;

	int i;
	for (i = 1; i < histories->n_histories; ++i)
	{
		min_length = MIN (min_length, histories->history[i].cursor.n_values);
	}

	return min_length;
}

int
histories_get_max_length (Histories *histories)
{
	int max_length = 0;

	int i;
	for (i = 0; i < histories->n_histories; ++i)
	{
		max_length = MAX (max_length, histories->history[i].cursor.n_values);
	}

	return max_length;
}

static void
print_element (FILE *file, Histories *histories, int history_index, int row)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return;
	}

	fprintf (file, "%f", row < history_get_length (histories, history_index) ? history_get_nth (histories, history_index, row) : 0);
}

static void
print_element_int (FILE *file, Histories *histories, int history_index, int row)
{
	if (histories->history[history_index].type != TYPE_INT)
	{
		fprintf (stderr, "%s: not an int history!\n", __func__);
		return;
	}

	fprintf (file, "%d", row < history_get_length (histories, history_index) ? history_get_nth_int (histories, history_index, row) : 0);
}

static void
histories_fprint (Histories *histories, FILE *file)
{
	HistoryPrintFunc print_func = histories->history[0].type == TYPE_DOUBLE ? print_element : print_element_int;
	int n_rows = histories_get_max_length (histories);
	int row;

	for (row = 0; row < n_rows; ++row)
	{
		int i;
		for (i = 0; i < histories->n_histories; ++i)
		{
			print_func (file, histories, i, row);

			if (i < histories->n_histories - 1)
			{
				fprintf (file, "\t");
			}
		}
		fprintf (file, "\n");
	}
}

void
histories_write_log (Histories *histories, char const *filename)
{
	FILE *file;

	file = fopen (filename, "w");
	if (!file)
	{
		fprintf (stderr, "Cannot open file \"%s\" for writing.\n", filename);
		return;
	}

	histories_fprint (histories, file);

	fclose (file);
}

void histories_print (Histories *histories)
{
	histories_fprint (histories, stdout);
}

double
history_sum (Histories *histories, int history_index)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];
	double sum = 0;
	
	int i;
	for (i = 0; i < history->cursor.n_values; ++i)
	{
		sum += ((double *) history->values)[i];
	}

	return sum;
}

double
history_average (Histories *histories, int history_index)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	if (history_get_length (histories, history_index) == 0)
	{
		return 0;
	}

	return history_sum (histories, history_index) / histories->history[history_index].cursor.n_values;
}

double
history_stdev (Histories *histories, int history_index)
{
	if (histories->history[history_index].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	History *history = &histories->history[history_index];
	double average = history_average (histories, history_index);
	double diff_square_sum = 0;
	int i;

	if (history->cursor.n_values < 2)
	{
		return 0;
	}

	for (i = 0; i < history->cursor.n_values; ++i)
	{
		double diff = ((double *) history->values)[i] - average;
		diff_square_sum += diff * diff;
	}

	return sqrt (1.0 / (history->cursor.n_values - 1.0) * diff_square_sum);
}

double
history_correlation (Histories *histories1, int index1, Histories *histories2, int index2)
{
	History *hist1 = &histories1->history[index1];
	History *hist2 = &histories2->history[index2];

	if (hist1->type != TYPE_DOUBLE || hist2->type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not double histories!\n", __func__);
		return 0;
	}

	int n = MIN (hist1->cursor.n_values, hist2->cursor.n_values);
	double average1 = history_average (histories1, index1);
	double average2 = history_average (histories2, index2);
	double diff_product_sum = 0;
	int i;

	for (i = 0; i < n; ++i)
	{
		diff_product_sum += (((double *) hist1->values)[i] - average1) * (((double *) hist2->values)[i] - average2);
	}

	return diff_product_sum / ((n - 1.0) * history_stdev (histories1, index1) * history_stdev (histories2, index2));
}

/* only for histories with identical max_n (because of the memcpy) */
static void
histories_copy (Histories *dest, Histories *src)
{
	int i;
	for (i = 0; i < src->n_histories; ++i)
	{
		memcpy (dest->history[i].values, src->history[i].values, src->history->cursor.max_n * type_size[src->history->type]);
		dest->history[i].cursor = src->history[i].cursor;
		dest->history[i].type = src->history[i].type;
	}
}

Histories *
histories_duplicate (Histories *histories)
{
	Histories *h = histories_new (histories->n_histories, histories->history->cursor.max_n);

	histories_copy (h, histories);

	return h;
}

void
histories_map (Histories *histories, HistoryMapFunc function)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return;
	}

	int i, j;

	for (i = 0; i < histories->n_histories; ++i)
	{
		for (j = 0; j < histories->history[i].cursor.n_values; ++j)
		{
			((double *) histories->history[i].values)[j] = function (((double *) histories->history[i].values)[j]);
		}
	}
}

Histories *
histories_diff (Histories *histories)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	int i, j;

	Histories *diff = histories_new (histories->n_histories, histories->history->cursor.max_n);
	
	for (i = 0; i < histories->n_histories; ++i)
	{
		for (j = 0; j < histories->history[i].cursor.n_values - 1; ++j)
		{
			history_add (diff, i, history_get_nth (histories, i, j + 1) - history_get_nth (histories, i, j));
		}
	}

	return diff;
}

/* /\* insertion sort implementation *\/ */
/* /\* length: number of values already stored in "values" *\/ */
/* static void */
/* insert_ascending (double *values, int length, double value) */
/* { */
/* 	int i; */
/* 	for (i = length; i > 0; --i) */
/* 	{ */
/* 		if (values[i - 1] <= value) */
/* 		{ */
/* 			break; */
/* 		} */
/* 		values[i] = values[i - 1]; */
/* 	} */
/* 	values[i] = value; */
/* } */

static Histories *
histories_concat (Histories *histories)
{
	int length = 0;
	int i;

	for (i = 0; i < histories->n_histories; ++i)
	{
		length += history_get_length (histories, i);
	}

	Histories *concat = histories_new (1, length);

	for (i = 0; i < histories->n_histories; ++i)
	{
		int j;
		
		for (j = 0; j < history_get_length (histories, i); ++j)
		{
			history_add (concat, 0, history_get_nth (histories, i, j));
		}
	}

	return concat;
}

static int
compare_doubles (const void *x, const void *y)
{
	if (*((double *) x) < *((double *) y))
	{
		return -1;
	}

	if (*((double *) x) > *((double *) y))
	{
		return 1;
	}

	return 0;
}

static double *
history_sorted (Histories *histories, int index)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}
	
	int n = histories->history[index].cursor.n_values;

	if (n == 0)
	{
		return NULL;
	}

	double *values = (double *) malloc (n * sizeof (double));

	memcpy (values, histories->history[index].values, n * sizeof (double));
	qsort (values, n, sizeof (double), compare_doubles);

	return values;
}

double
history_median (Histories *histories, int index)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	int n = histories->history[index].cursor.n_values;

	if (n == 0)
	{
		return 0;
	}

	double *values = history_sorted (histories, index);

	double median;

	if (n % 2)
	{
		median = values[n / 2];
	}
	else
	{
		median = (values[n / 2 - 1] + values[n / 2]) / 2;
	}

	free (values);

	return median;
}

static Histories *
histories_summary (Histories *histories, HistorySummaryFunc function)
{
	int count = histories_get_count (histories);
	Histories *out = histories_new (1, count);
	int i;

	for (i = 0; i < count; ++i)
	{
		history_add (out, 0, function (histories, i));
	}

	return out;
}

Histories *
histories_sum (Histories *histories)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	return histories_summary (histories, history_sum);
}

Histories *
histories_average (Histories *histories)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	return histories_summary (histories, history_average);
}

Histories *
histories_stdev (Histories *histories)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	return histories_summary (histories, history_stdev);
}

Histories *
histories_median (Histories *histories)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	return histories_summary (histories, history_median);
}

double
histories_correlation (Histories *hist1, Histories *hist2)
{
	if (hist1->history[0].type != TYPE_DOUBLE || hist2->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not double histories!\n", __func__);
		return 0;
	}

	Histories *concat1 = histories_concat (hist1);
	Histories *concat2 = histories_concat (hist2);

	double correlation = history_correlation (concat1, 0, concat2, 0);

	histories_delete (concat1);
	histories_delete (concat2);

	return correlation;
}

Histories *
histories_select (Histories *histories, int index_from, int index_to)
{
	index_from = MAX (0, index_from);
	index_to = MIN (histories->n_histories - 1, index_to);

	int n_hist = index_to - index_from + 1;

	if (n_hist == 0)
	{
		return NULL;
	}
	
	Histories *out;

	if (histories->history[0].type == TYPE_DOUBLE)
	{
		out = histories_new (n_hist, histories->history[0].cursor.max_n);
	}
	else
	{
		out = histories_new_int (n_hist, histories->history[0].cursor.max_n);
	}

	int i;
	for (i = 0; i < n_hist; ++i)
	{
		int input_index = index_from + i;

		int j;
		for (j = 0; j < history_get_length (histories, input_index); ++j)
		{
			if (histories->history[i].type == TYPE_DOUBLE)
			{
				history_add (out, i, history_get_nth (histories, input_index, j));
			}
			else
			{
				history_add_int (out, i, history_get_nth_int (histories, input_index, j));
			}
		}
	}

	return out;
}

int
histories_get_count (Histories *histories)
{
	return histories->n_histories;
}

double 
history_quantile (Histories *histories,
                  int        index,
                  double     quantile)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return 0;
	}

	int n = histories->history[index].cursor.n_values;

	if (n == 0)
	{
		return 0;
	}

	double *values = history_sorted (histories, index);

	int ind = CLIP (lround (quantile * (n - 1)), 0, n - 1);
	double result = values[ind];

	free (values);

	return result;
}


Histories *histories_quantiles (Histories *histories,
                                double    *quantiles,
                                int        n_quantiles)
{
	if (histories->history[0].type != TYPE_DOUBLE)
	{
		fprintf (stderr, "%s: not a double history!\n", __func__);
		return NULL;
	}

	int count = histories_get_count (histories);
	Histories *out = histories_new (count, n_quantiles);
	int i;

	for (i = 0; i < count; ++i)
	{
		int j;
		
		for (j = 0; j < n_quantiles; ++j)
		{
			history_add (out, i, history_quantile (histories, i, quantiles[j]));
		}
	}

	return out;
}
