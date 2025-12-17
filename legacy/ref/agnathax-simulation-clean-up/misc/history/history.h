#ifndef __HISTORY_H
#define __HISTORY_H

#include "../defines.h"

typedef struct {
	int max_n;
	int n_values;
	int cursor;
} HistoryCursor;

typedef struct _Histories Histories;

typedef double (*HistoryMapFunc) (double);

// #ifdef __cplusplus
// extern "C" {
// #endif

	/* cursor functions */
	int history_cursor_add (HistoryCursor *cursor);
	int history_cursor_nth (HistoryCursor *cursor, int nth);
	int history_cursor_nth_last (HistoryCursor *cursor, int nth);

	/* generic functions */
	int history_get_length (Histories *histories, int history_index);
	int histories_get_count (Histories *histories);
	void histories_delete (Histories *histories);
	Histories * histories_duplicate (Histories *histories);
	int histories_get_min_length (Histories *histories);
	int histories_get_max_length (Histories *histories);
	void histories_write_log (Histories *histories, char const *filename);
	void histories_print (Histories *histories);
	Histories * histories_select (Histories *histories, int index_from, int index_to);

	/* int histories */
	Histories * histories_new_int (int n_variables, int max_n_rows);
	void histories_add_int (Histories *histories, int *values);
	void history_add_int (Histories *histories, int history_index, int value);
	int history_get_nth_int (Histories *histories, int history_index, int nth); /* first: nth = 0 */
	int history_get_last_int (Histories *histories, int history_index);
	int history_get_nth_last_int (Histories *histories, int history_index, int nth); /* last: nth = 0 */

	/* double histories */
	Histories * histories_new (int n_variables, int max_n_rows);
	void histories_add (Histories *histories, double *values);
	void history_add (Histories *histories, int history_index, double value);
	double history_get_nth (Histories *histories, int history_index, int nth); /* first: nth = 0 */
	double history_get_last (Histories *histories, int history_index);
	double history_get_nth_last (Histories *histories, int history_index, int nth); /* last: nth = 0 */
	double history_sum (Histories *histories, int history_index);
	double history_average (Histories *histories, int history_index);
	double history_stdev (Histories *histories, int history_index);
	double history_correlation (Histories *hist1, int index1, Histories *hist2, int index2);
	void histories_map (Histories *histories, HistoryMapFunc function);
	Histories * histories_diff (Histories *histories);
	double history_median (Histories *histories, int index);
	double history_quantile (Histories *histories, int index, double quantile);
	Histories * histories_sum (Histories *histories);
	Histories * histories_average (Histories *histories);
	Histories * histories_stdev (Histories *histories);
	Histories * histories_median (Histories *histories);
	Histories * histories_quantiles (Histories *histories, double *quantiles, int n_quantiles);
	double histories_correlation (Histories *hist, Histories *hist2);

	/* this one puts floats into double histories */
	void histories_addf (Histories *histories, float *values);

	/* We give here a definition of the cursor functions only for inlining, so that
	 * the cursor code can be used wihout linking history.o */
	extern inline int
	history_cursor_add (HistoryCursor *cursor)
	{
		cursor->n_values = MIN (cursor->n_values + 1, cursor->max_n);
		cursor->cursor = (cursor->cursor + 1) % cursor->max_n;

		return cursor->cursor;
	}

	extern inline int
	history_cursor_nth_last (HistoryCursor *cursor,
	                         int            nth)
	{
		int history_index = cursor->cursor - nth;
	
		if (history_index < 0)
		{
			history_index += cursor->max_n;
		}
	
		return history_index;
	}

	extern inline int
	history_cursor_nth (HistoryCursor *cursor,
	                    int            nth)
	{

		return (cursor->cursor + 1 + nth) % MIN (cursor->n_values, cursor->max_n);
	}

// #ifdef __cplusplus
// }
// #endif

#endif
