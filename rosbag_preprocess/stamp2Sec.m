function out = stamp2Sec(header_in)
    out = double(header_in.stamp.sec) + double(header_in.stamp.nanosec) * 1e-9;
end