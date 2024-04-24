import unittest
from unittest.mock import mock_open, patch
from pathlib import Path
import numpy as np
from aos.events.logging.plot_logger_profile import SeriesDetail, parse_csv_file, create_binned_data

# Mock CSV data as a string
# The actual column names are "channel_name, channel_type, encode_duration_ns, encoding_start_time_ns, message_time_s".
mock_csv = """# It doesn't matter what's in this comment. The column names are put here for your reference.
/test/channel/name1,aos.test.channel.name1,1000000,123456789,10
/test/channel/name2,aos.test.channel.name1,2000000,123456789,20
/test/channel/name3,aos.test.channel.name2,3000000,123456789,30
"""


class TestLogParser(unittest.TestCase):

    def test_parse_csv_file(self):
        # Expected result after parsing the mock CSV, using SeriesDetail instances
        expected_result = {
            'aos.test.channel.name1':
            SeriesDetail(encode_times_ms=[1.0, 2.0],
                         event_loop_times_s=[10.0, 20.0]),
            'aos.test.channel.name2':
            SeriesDetail(encode_times_ms=[3.0], event_loop_times_s=[30.0])
        }

        # Use 'mock_open' to simulate file opening and reading
        with patch('builtins.open', mock_open(read_data=mock_csv)):
            # Parse the data assuming the mock file path is 'dummy_path.csv'
            result = parse_csv_file(Path('dummy_path.csv'),
                                    max_lines=None,
                                    start_time=None,
                                    end_time=None)

            # Check if the parsed data matches the expected result
            self.assertEqual(result, expected_result)


class TestCreateBinnedData(unittest.TestCase):

    def test_create_binned_data_with_other_category(self):
        # Setup input data with three types
        data_by_type = {
            'Type1':
            SeriesDetail(encode_times_ms=[10.0, 20.0, 30.0],
                         event_loop_times_s=[13.0, 23.0, 33.0]),
            'Type2':
            SeriesDetail(encode_times_ms=[15.0, 25.0, 35.0],
                         event_loop_times_s=[18.0, 28.0, 38.0]),
            'Type3':
            SeriesDetail(encode_times_ms=[5.0, 10.0, 15.0],
                         event_loop_times_s=[8.0, 19.0, 29.0])
        }

        # Choose top_n less than the number of types. This will cause Type3's data to go into 'other'
        top_n = 2

        # Sort the types by total encoding times to determine top types
        expected_top_types = ['Type2', 'Type1']

        bins = np.linspace(8, 38, len(data_by_type) + 1)

        # Expected output for 3 bins. The bin bounds were measured to be 18.0, 28.0, and 38.0.
        expected_bins = [
            {
                "bin_range": f"{bins[0]:.2f} - {bins[1]:.2f}",
                "message_encode_times": {
                    "Type1": 10.0,
                    "Type2": 0,
                    "other": 5.0
                }
            },
            {
                "bin_range": f"{bins[1]:.2f} - {bins[2]:.2f}",
                "message_encode_times": {
                    "Type1": 20.0,
                    "Type2": 15.0,
                    "other": 10.0
                }
            },
            {
                "bin_range": f"{bins[2]:.2f} - {bins[3]:.2f}",
                "message_encode_times": {
                    "Type1": 30.0,
                    "Type2": 60.0,
                    "other": 15.0
                }
            },
        ]

        # Call the function under test
        result = create_binned_data(data_by_type,
                                    num_bins=3,
                                    top_n=len(expected_top_types))

        # Check the top type names and order
        self.assertEqual(result.top_type_names, expected_top_types)

        # Check each bin's range and encode times
        for idx, bin_data in enumerate(result.bins):
            self.assertEqual(bin_data.bin_range,
                             expected_bins[idx]["bin_range"])
            self.assertEqual(bin_data.message_encode_times,
                             expected_bins[idx]["message_encode_times"])


if __name__ == '__main__':
    unittest.main()
