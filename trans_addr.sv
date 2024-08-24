//DMW
`define PLV0      0
`define PLV3      3 
`define DMW_MAT   5:4
`define PSEG      27:25
`define VSEG      31:29

module trans_addr//未实现TLB，不包含查询TLB表项，只针对串口进行处理，面向测试编程了哈哈
#(
    parameter TLBNUM = 32
)
(
    input                  clk                  ,
    input                  rst                  ,
    icache_transaddr       icache2transaddr     ,
    dcache_transaddr       dcache2transaddr     ,
    csr_tlb                csr2tlb              , 
    output                 iuncache
);

logic pg_mode;
logic da_mode;
assign pg_mode = !csr2tlb.csr_da &&  csr2tlb.csr_pg;
assign da_mode =  csr2tlb.csr_da && !csr2tlb.csr_pg;
logic data_dmw0_en,data_dmw1_en;
assign data_dmw0_en = ((csr2tlb.csr_dmw0[`PLV0] & csr2tlb.csr_plv == 2'd0) | (csr2tlb.csr_dmw0[`PLV3] & csr2tlb.csr_plv == 2'd3)) & (dcache2transaddr.data_vaddr[31:29] == csr2tlb.csr_dmw0[`VSEG]) & pg_mode;
assign data_dmw1_en = ((csr2tlb.csr_dmw1[`PLV0] & csr2tlb.csr_plv == 2'd0) | (csr2tlb.csr_dmw1[`PLV3] & csr2tlb.csr_plv == 2'd3)) & (dcache2transaddr.data_vaddr[31:29] == csr2tlb.csr_dmw1[`VSEG]) & pg_mode;

//存一拍信号
reg  [31:0] inst_vaddr_buffer_a  ;
reg  [31:0] inst_vaddr_buffer_b  ;
reg  [31:0] data_vaddr_buffer    ;

always @(posedge clk) begin
    inst_vaddr_buffer_a <= icache2transaddr.inst_vaddr_a;
    inst_vaddr_buffer_b <= icache2transaddr.inst_vaddr_b;
    data_vaddr_buffer <= dcache2transaddr.data_vaddr;
end

wire [31:0] data_paddr;

logic [4:0] data_offset;
logic [6:0] data_index;
logic [19:0] data_tag;

assign data_paddr = (pg_mode & data_dmw0_en & !dcache2transaddr.cacop_op_mode_di) ? {csr2tlb.csr_dmw0[`PSEG], dcache2transaddr.data_vaddr[28:0]} : 
                    (pg_mode & data_dmw1_en & !dcache2transaddr.cacop_op_mode_di) ? {csr2tlb.csr_dmw1[`PSEG], dcache2transaddr.data_vaddr[28:0]} : dcache2transaddr.data_vaddr;

assign data_offset = dcache2transaddr.data_vaddr[4:0];
assign data_index  = dcache2transaddr.data_vaddr[11:5];
assign data_tag    = data_paddr[31:12];

always_ff @( posedge clk) begin
    dcache2transaddr.ret_data_paddr <= {data_tag,data_index,data_offset};
end
assign icache2transaddr.ret_inst_paddr_a=inst_vaddr_buffer_a;
assign icache2transaddr.ret_inst_paddr_b=inst_vaddr_buffer_b;

assign icache2transaddr.uncache = 0;
assign iuncache = icache2transaddr.uncache;

always_ff @( posedge clk ) begin
    if(dcache2transaddr.data_vaddr[31:16]==16'hbfaf)dcache2transaddr.uncache<=1'b1;
    else dcache2transaddr.uncache<=1'b0;
end
endmodule