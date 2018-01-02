from datetime import datetime
import os, sys

def get_modification_date(file_name):
    try:
        mtime = os.path.getmtime(file_name)
    except OSError:
        print '\033[91mCannot get modification date from %s \033[0m' % file_name
        return None

    return datetime.fromtimestamp(mtime)


def parse_start_end(args, current):
    try:
        start_time = datetime.strptime(args["<start_time>"], "%H:%M")
        if args["<end_time>"] == "now":
            end_time = current
        else:
            end_time = datetime.strptime(args["<end_time>"], "%H:%M")

        start_date = current
        end_date = current

        start_combined = datetime.combine(start_date.date(), start_time.time())
        end_combined = datetime.combine(end_date.date(), end_time.time())

        if start_combined > end_combined:
            raise ValueError("Specified start is larger than end!")

    except ValueError as e:
        print '\033[91mInvalid input: %s \033[0m' % e
        sys.exit(1)

    return start_combined, end_combined
