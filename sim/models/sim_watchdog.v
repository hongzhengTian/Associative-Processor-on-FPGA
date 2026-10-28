`timescale 1ps/1ps
// Second top-level module for the open-source flow: ends a run that never
// reaches finish_flag (e.g. a hang in the DDR handshake) instead of letting the
// simulator spin forever. Override the limit with +timeout_ns=<n>.
module sim_watchdog;
    integer timeout_ns = 300000;
    initial begin
        if ($value$plusargs("timeout_ns=%d", timeout_ns)) ;
        #(timeout_ns * 1000.0);
        $display("TIMEOUT: finish_flag not seen after %0d ns", timeout_ns);
        $display("simulation FAILED");
        $finish;
    end
endmodule
