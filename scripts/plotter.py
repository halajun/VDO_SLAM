import csv
import argparse 

class CSVLoader(object):

    def __init__(self, path_to_csv):
        self.rows = []
        self._open_file(path_to_csv)

    def _open_file(self, path_to_csv):
        print("Loading CSV file - {}".format(path_to_csv))
        try:
            with open(path_to_csv) as csv_file:
                reader = csv.DictReader(csv_file, delimiter=",")
                for row in reader:
                    self.rows.append(row)
                    # print(row)
        except Exception as e:
            print(str(e))


def main():
    parser = argparse.ArgumentParser(description='')
    parser.add_argument("csv_file", type=str, help="Full path to csv file to parse")
    args = parser.parse_args()

    if not args.csv_file:
        print("No file")
        return 0
    else:
        loader = CSVLoader(args.csv_file)


if __name__ == '__main__':
    import sys
    sys.exit(main())