#include <stdio.h>
#include <stdlib.h>

#include "history.h"

double
times2 (double x)
{
	return 2 * x;
}

int
main ()
{
	Histories *histories = histories_new (2, 10);

	int i;

	for (i = 0; i < 5; ++i)
	{
		history_add (histories, 0, i);
		history_add (histories, 1, -i);
	}

	history_add (histories, 1, 1000);

	printf ("length: %d %d (min %d, max %d)\n",
	        history_get_length (histories, 0),
	        history_get_length (histories, 1),
	        histories_get_min_length (histories),
	        histories_get_max_length (histories));

	for (i = 0 ; i < 7; ++i)
	{
		double x[2];
		x[0] = 10 * i;
		x[1] = -10 * i;
		histories_add (histories, x);
	}

	printf ("length: %d %d (min %d, max %d)\n",
	        history_get_length (histories, 0),
	        history_get_length (histories, 1),
	        histories_get_min_length (histories),
	        histories_get_max_length (histories));

	printf ("last: %f %f\n",
	        history_get_last (histories, 0),
	        history_get_last (histories, 1));

	int length = histories_get_max_length (histories);

	for (i = 0; i < length; ++i)
	{
		printf ("%dth: %f %f\n",
		        i,
		        history_get_nth (histories, 0, i),
		        history_get_nth (histories, 1, i));
	}


	for (i = 0; i < length; ++i)
	{
		printf ("last %dth: %f %f\n",
		        i,
		        history_get_nth_last (histories, 0, i),
		        history_get_nth_last (histories, 1, i));
	}

	printf ("sum: %f %f\n",
	        history_sum (histories, 0),
	        history_sum (histories, 1));

	printf ("mean: %f %f\n",
	        history_average (histories, 0),
	        history_average (histories, 1));

	Histories *h_copy = histories_duplicate (histories);

	length = histories_get_max_length (h_copy);
	histories_map (h_copy, times2);
	puts ("\ncoppied, * 2:");
	for (i = 0; i < length; ++i)
	{
		printf ("%dth: %f %f\n",
		        i,
		        history_get_nth (h_copy, 0, i),
		        history_get_nth (h_copy, 1, i));
	}
	printf ("sum: %f %f\n",
	        history_sum (h_copy, 0),
	        history_sum (h_copy, 1));

	Histories *hsum = histories_sum (h_copy);
	printf ("sum merged: %f\n", history_sum (hsum, 0));

	Histories *hdiff = histories_diff (h_copy);
	length = histories_get_max_length (hdiff);
	puts ("\ndiff:");
	for (i = 0; i < length; ++i)
	{
		printf ("%dth: %f %f\n",
		        i,
		        history_get_nth (hdiff, 0, i),
		        history_get_nth (hdiff, 1, i));
	}

	printf ("\nmedians: %f, %f\n", history_median (histories, 0), history_median (histories, 1));


	histories_write_log (histories, "log_history_test");

	Histories *h4 = histories_new (4, 10);
	for (i = 0; i < 10; ++i)
	{
		history_add (h4, 0, i);
		history_add (h4, 1, i * 10);
		history_add (h4, 2, -i);
		history_add (h4, 3, -i * 10);
	}

	Histories *selected = histories_select (h4, 2, 3);

	histories_write_log (selected, "log_selected");

	int const n_rows = 1000;
	int const n_hist = 3;
	Histories *random_data = histories_new (n_hist, n_rows);
	
	for (i = 0; i < n_rows; ++i)
	{
		double rnd[n_hist];

		int j;

		for (j = 0; j < n_hist; ++j)
		{
			rnd[j] = RAND(0, 1);
			printf ("%f\t", rnd[j]);
		}
		printf ("\n");

		histories_add (random_data, rnd);
	}
	
	double q[] = {0.1, 0.99};
	printf ("Quantile %f - %f, %f - %f, %f - %f\n",
	        history_quantile (random_data, 0, q[0]), history_quantile (random_data, 0, q[1]),
	        history_quantile (random_data, 1, q[0]), history_quantile (random_data, 1, q[1]),
	        history_quantile (random_data, 2, q[0]), history_quantile (random_data, 2, q[1]));

	Histories *quantiles = histories_quantiles (random_data, q, 2);

	histories_print (quantiles);

	histories_delete (quantiles);
	histories_delete (random_data);
	histories_delete (h_copy);
	histories_delete (selected);
	histories_delete (h4);
	histories_delete (hsum);
	histories_delete (hdiff);
	histories_delete (histories);

	return 0;
}
