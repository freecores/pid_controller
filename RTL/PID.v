/* PID controller

sigma=Ki*e(n)+sigma
u(n)=(Kp+Kd)*e(n)+sigma+Kd*(-e(n-1))

Data width of Wishbone slave port can be can be toggled between 64-bit, 32-bit and 16-bit.
Address width of Wishbone slave port can be can be modified by changing parameter adr_wb_nb.

Wishbone compliant
Work as Wishbone slave, support Classic standard SINGLE/BLOCK READ/WRITE Cycle

registers or wires
[15:0]kp,ki,kd,sp,pv;	can be both read and written through Wishbone interface, address: 0x0, 0x4, 0x8, 0x12, 0x16
[15:0]kpd;		read only through Wishbone interface, address: 0x20
[15:0]err[0:1];		read only through Wishbone interface, address: 0x24, 0x28
[15:0]mr,md;		not accessable through Wishbone interface
[31:0]p,b;			not accessable through Wishbone interface
[31:0]un,sigma;		read only through Wishbone interface, address: 0x32, 0x36


[4:0]of;			overflow register, read only through Wishbone interface, address: 0x40
of[0]==1	:	kpd overflow
of[1]==1	:	err[0] overflow
of[2]==1	:	err[1] overflow
of[3]==1	:	un overflow
of[4]==1	:	sigma overflow
[0:15]rl;			read lock, when asserted corelated reagister can not be read through Wishbone interface
[0:7]wl;			write lock, when asserted corelated reagister can not be written through Wishbone interface



*/

`include "PID_defines.v"

module	PID #(
`ifdef wb_16bit
parameter	wb_nb=16,
`endif
`ifdef wb_32bit
parameter	wb_nb=32,
`endif
`ifdef wb_64bit
parameter	wb_nb=64,
`endif
		adr_wb_nb=16
)(
input	i_clk,
input	i_rst,	//reset when low
//Wishbone Slave port
input	i_wb_cyc,
input	i_wb_stb,
input	i_wb_we,
input	[adr_wb_nb-1:0]i_wb_adr,
input	[wb_nb-1:0]i_wb_data,
output	o_wb_ack,
output	[wb_nb-1:0]o_wb_data,

//u(n) output
output	[31:0]o_un,
output	o_valid
);

parameter	kp_adr		=	0,
		ki_adr		=	1,
		kd_adr		=	2,
		sp_adr		=	3,
		pv_adr		=	4,
		kpd_adr		=	5,
		err_0_adr		=	6,
		err_1_adr		=	7,
		un_adr		=	8,
		sigma_adr	=	9,
		of_adr		=	10;

reg	[15:0]kp,ki,kd,sp,pv;
reg	wlkp,wlki,wlkd,wlsp,wlpv;	// write lock, if one is high the relevant reg is not writeable

wire	[0:7]wl={wlkp,wlki,wlkd,wlsp,wlpv,3'h0};	// write lock

reg	wack;	//write acknowledged

wire	[2:0]adr;
`ifdef wb_16bit
assign	adr=i_wb_adr[3:1];
`endif
`ifdef wb_32bit
assign	adr=i_wb_adr[4:2];
`endif
`ifdef wb_64bit
assign	adr=i_wb_adr[5:3];
`endif

wire	[3:0]adr_1;
`ifdef	wb_32bit
assign	adr_1=i_wb_adr[5:2];
`endif
`ifdef	wb_16bit
assign	adr_1=i_wb_adr[4:1];
`endif
`ifdef	wb_64bit
assign	adr_1=i_wb_adr[6:3];
`endif


wire	we;	// write enable
assign	we=i_wb_cyc&i_wb_we&i_wb_stb;
wire	re;	//read enable
assign	re=i_wb_cyc&(~i_wb_we)&i_wb_stb;

reg	state_0;  //state machine No.1's state register

wire	adr_check_1;
`ifdef	wb_32bit
assign	adr_check_1=i_wb_adr[adr_wb_nb-1:6]==0;
`endif
`ifdef	wb_16bit
assign	adr_check_1=i_wb_adr[adr_wb_nb-1:5]==0;
`endif
`ifdef	wb_64bit
assign	adr_check_1=i_wb_adr[adr_wb_nb-1:7]==0;
`endif

wire	adr_check;	//check address's correctness
`ifdef wb_16bit
assign	adr_check=i_wb_adr[4]==0&&adr_check_1;
`endif
`ifdef wb_32bit
assign	adr_check=i_wb_adr[5]==0&&adr_check_1;
`endif
`ifdef wb_64bit
assign	adr_check=i_wb_adr[6]==0&&adr_check_1;
`endif

 //state machine No.1
always@(posedge i_clk or posedge i_rst)
	if(i_rst)begin
		state_0<=0;
		wack<=0;
		kp<=0;
		ki<=0;
		kd<=0;
		sp<=0;
		pv<=0;
		
	end
	else	begin
		if(wack&&(!i_wb_stb)) wack<=0;
		case(state_0)
		0:	begin
			if(we&&(!wack)) state_0<=1;
		end
		1:	begin
			if(adr_check)begin
				if(!wl[adr])begin
					wack<=1;
					state_0<=0;
					case(adr)
					0:	begin
						kp<=i_wb_data[15:0];
					end
					1:	begin
						ki<=i_wb_data[15:0];
					end
					2:	begin
						kd<=i_wb_data[15:0];
					end
					3:	begin
						sp<=i_wb_data[15:0];
					end
					4:	begin
						pv<=i_wb_data[15:0];
					end
					endcase

				end
			end
			else begin
				 wack<=1;
				state_0<=0;
			end
		end
		endcase
	end


 //state machine No.2
reg	[11:0]state_1;

wire	update_kpd;
assign	update_kpd=wack&&(~adr[2])&&(~adr[0])&&adr_check;	//adr==0||adr==2

wire	update_esu;	//update e(n), sigma and u(n)
assign	update_esu=wack&&(adr==4)&&adr_check;

reg	rlkpd;	// read lock
reg	rlerr_0;	// read lock
reg	rlerr_1;	// read lock
reg	rla;	// read lock
reg	rlsigma;	// read lock
reg	rlof;	// read lock

reg	[4:0]of;
reg	[15:0]kpd;
reg	[15:0]err[0:1];

wire	[15:0]mr,md;

reg	[31:0]p;
reg	[31:0]a,sigma,un;

reg 	start;	//start signal for multiplier

reg	[1:0]mr_index;
reg	md_index;
assign	mr=	mr_index==1?kpd:
		mr_index==2?kd:ki;
assign	md=	md_index?err[1]:err[0];

reg	cout;
wire	cin;
wire	[31:0]sum;
wire	[31:0]product;

wire	of_addition[0:1];
assign	of_addition[0]=(p[15]&&a[15]&&(!sum[15]))||((!p[15])&&(!a[15])&&sum[15]);
assign	of_addition[1]=(p[31]&&a[31]&&(!sum[31]))||((!p[31])&&(!a[31])&&sum[31]);

always@(posedge i_clk or posedge i_rst)
	if(i_rst)begin
		state_1<=12'b000000000001;
		wlkp<=0;
		wlki<=0;
		wlkd<=0;
		wlsp<=0;
		wlpv<=0;
		rlkpd<=0;	
		rlerr_0<=0;
		rlerr_1<=0;
		rla<=0;
		rlsigma<=0;
		rlof<=0;
		of<=0;
		kpd<=0;
		err[0]<=0;
		err[1]<=0;
		p<=0;
		a<=0;
		sigma<=0;
		un<=0;
		start<=0;
		mr_index<=0;
		md_index<=0;
		cout<=0;
	end
	else begin
		case(state_1)
		12'b000000000001:	begin
			if(update_kpd)begin
				state_1<=12'b000000000010;
				wlkp<=1;
				wlkd<=1;	
				wlpv<=1;
				rlkpd<=1;
				rlof<=1;
			end
			else if(update_esu)begin
				state_1<=12'b000000001000;
				wlkp<=1;
				wlki<=1;
				wlkd<=1;
				wlsp<=1;
				wlpv<=1;
				rlkpd<=1;	
				rlerr_0<=1;
				rlerr_1<=1;
				rla<=1;
				rlsigma<=1;
				rlof<=1;
			end
		end
		12'b000000000010:	begin
			p<={{16{kp[15]}},kp};
			a<={{16{kd[15]}},kd};
			state_1<=12'b000000000100;
		end
		12'b000000000100:	begin
			kpd<=sum[15:0];
			wlkp<=0;
			wlkd<=0;	
			wlpv<=0;
			rlkpd<=0;
			rlof<=0;
			of[0]<=of_addition[0];
			state_1<=12'b000000000001;
		end
		12'b000000001000:	begin
			p<={{16{~err[0][15]}},~err[0]};
			a<={31'b0,1'b1};
			state_1<=12'b000000010000;
		end
		12'b000000010000:	begin
			err[1]<=sum[15:0];
			of[2]<=of[1];
			p<={{16{sp[15]}},sp};
			a<={{16{~pv[15]}},~pv};
			cout<=1;
			state_1<=12'b000000100000;
		end		
		12'b000000100000:	begin
			err[0]<=sum[15:0];
			of[1]<=of_addition[0];
			cout<=0;
			start<=1;
			state_1<=12'b000001000000;
		end
		12'b000001000000:	begin
			mr_index<=1;
			state_1<=12'b000010000000;
		end
		12'b000010000000:	begin
			mr_index<=2;
			md_index<=1;
			state_1<=12'b000100000000;
		end
		12'b000100000000:	begin
			mr_index<=0;
			md_index<=0;
			start<=0;
			p<=product;
			a<=sigma;
			state_1<=12'b001000000000;
		end
		12'b001000000000:	begin
			a<=sum;
			sigma<=sum;
			of[3]<=of[4]|of_addition[1];
			of[4]<=of[4]|of_addition[1];
			p<=product;
			state_1<=12'b010000000000;
			
		end
		12'b010000000000:	begin
			a<=sum;
			of[3]<=of[3]|of_addition[1];
			p<=product;			
			state_1<=12'b100000000000;						
		end
		12'b100000000000:	begin
			un<=sum;
			of[3]<=of[3]|of_addition[1];
			state_1<=12'b000000000001;
			wlkp<=0;
			wlki<=0;
			wlkd<=0;
			wlsp<=0;
			wlpv<=0;
			rlkpd<=0;	
			rlerr_0<=0;
			rlerr_1<=0;
			rla<=0;
			rlsigma<=0;
			rlof<=0;			
		end
		endcase
	end


wire	ready;
multiplier_16x16bit_pipelined	multiplier_16x16bit_pipelined(
i_clk,
~i_rst,
start,
md,
mr,
product,
ready
);

adder_32bit	adder_32bit_0(
a,
p,
cout,
sum,
cin
);


wire	[wb_nb-1:0]rdata[0:15];	//wishbone read data array
`ifdef	wb_16bit
assign	rdata[0]=kp;
assign	rdata[1]=ki;
assign	rdata[2]=kd;
assign	rdata[3]=sp;
assign	rdata[4]=pv;
assign	rdata[5]=kpd;
assign	rdata[6]=err[0];
assign	rdata[7]=err[1];
assign	rdata[8]=un[15:0];
assign	rdata[9]=sigma[15:0];
assign	rdata[10]={11'b0,of};
`endif

`ifdef	wb_32bit
assign	rdata[0]={{16{kp[15]}},kp};
assign	rdata[1]={{16{ki[15]}},ki};
assign	rdata[2]={{16{kd[15]}},kd};
assign	rdata[3]={{16{sp[15]}},sp};
assign	rdata[4]={{16{pv[15]}},pv};
assign	rdata[5]={{16{kpd[15]}},kpd};
assign	rdata[6]={{16{err[0][15]}},err[0]};
assign	rdata[7]={{16{err[1][15]}},err[1]};
assign	rdata[8]=un;
assign	rdata[9]=sigma;
assign	rdata[10]={27'b0,of};
`endif

`ifdef	wb_64bit
assign	rdata[0]={{48{kp[15]}},kp};
assign	rdata[1]={{48{ki[15]}},ki};
assign	rdata[2]={{48{kd[15]}},kd};
assign	rdata[3]={{48{sp[15]}},sp};
assign	rdata[4]={{48{pv[15]}},pv};
assign	rdata[5]={{48{kpd[15]}},kpd};
assign	rdata[6]={{48{err[0][15]}},err[0]};
assign	rdata[7]={{48{err[1][15]}},err[1]};
assign	rdata[8]={{32{un[31]}},un};
assign	rdata[9]={{32{sigma[31]}},sigma};
assign	rdata[10]={59'b0,of};
`endif

assign	rdata[11]=0;
assign	rdata[12]=0;
assign	rdata[13]=0;
assign	rdata[14]=0;
assign	rdata[15]=0;


wire	[0:15]rl;
assign	rl={5'b0,rlkpd,rlerr_0,rlerr_1,rla,rlsigma,rlof,5'b0};

wire	rack;	// wishbone read acknowledged
assign	rack=(re&adr_check_1&(~rl[adr_1]))|(re&(~adr_check_1));

assign	o_wb_ack=(wack|rack)&i_wb_stb;

assign	o_wb_data=adr_check_1?rdata[adr_1]:0;
assign	o_un=un;
assign	o_valid=~rla;


endmodule