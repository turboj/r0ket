#!/usr/bin/perl
#
# vim:set ts=4 sw=4:

use strict;

use IO::Select;
package r0ket;

use Digest::CRC qw(crcccitt);
use POSIX qw(strftime);
use Time::HiRes;

our $verbose=0;
our $bridge; # Open device
our $quiet=0;
our $timediff=60*60*2;

### Utility
sub sprint{
    return join("",map {
		if (ord($_)>30 && ord($_)<127){
			$_;
		}else{
#			"[x".unpack("H*",$_)."]";
			"\\".unpack("C",$_);
		}
        }split(//,shift));
};

sub hprint{
    return unpack("H*",shift);
};

sub flagsstr {
    my $in=shift;
    my @f;
    my $f=1;
    for (@_){
        if($in & $f){
            push @f,$_;
        };
        $f*=2;
    };
    return join(",",@f);
};




### Nickname/beacon helper functions
our %beacon;
sub readbeacon{
       return if( ! -f "BEACON" );
       open(B,"<","BEACON") || die "open: $!";
       while(<B>){
           /(\w+)\s+(.*)/ && do {
               $beacon{$1}=$2;
           };
       };
       close(B);
};
sub getbeacon{
    my $b=shift;
    if(!$beacon{$b}){
        return "";
    }else{
        return $beacon{$b};
    };
};
sub resolvebeacon{
    my $b=shift;
    if(!$beacon{$b}){
        return $b;
    }else{
        return "$b ($beacon{$b})";
    };
};
sub addbeacon{
    my($b,$n)=@_;
    if(!$beacon{$b}){
        $beacon{$b}=$n;
    };
};
sub writebeacon{
    open(B,">","BEACON") || die "write: $!";
    for(sort keys %beacon){
        print B "$_ $beacon{$_}\n";
    };
    close(B);
};

### Packet mgmt

our $buffer;
our $firstpkt=2;
sub get_packet{
    sub _get_bytes{
        my $rr;
        sysread($bridge,$rr,1024);
        if(length($rr)<=1){
            select(undef,undef,undef,0.05);
        };
        $buffer.=$rr;
    };

    my $cnt=0;
    while(1){
        if(length($buffer)<2){
            _get_bytes();
        }elsif($buffer !~ /^\\[12]/){
            $buffer=~s/^(.[^\\]*)//s;
            if($firstpkt){
                $firstpkt--;
            }else{
                print STDERR "Unparseable stuff: <",sprint($1),">\n" if(!$quiet);
            };
        }elsif ($buffer =~ s/^\\2\\0//s){
            return 'ack'; # In-band signalling. Evil %)
        }elsif ($buffer =~ s/^\\1(.*?)\\0//s){
            my $str=$1;
            $str=~s/\\\\/\\/g; # dequote
            return $str;
        }else{
            _get_bytes();
        };
        if(++$cnt%100 == 0){
            if(!$quiet){
                print STDERR "No packets for 5 seconds?\n";
            };
        };
    };
};

sub rest{
    if(length($buffer)>0){
        print "rest: <", sprint($buffer), ">\n" if(!$quiet);
    };
};

### Pkt beautify
sub nice_mesh{
    my $pkt=shift;
    my $out;
    my $type=substr($pkt,0,1);
#    next if(defined $arg && $arg ne $i);
    $out->{type}=$type;
    $out->{string}="[$type]";
    $out->{generation}=unpack("C",substr($pkt,1,1));
    $out->{string}.= " g=".$out->{generation};
    if($type eq "T"){
        $out->{time}=   unpack("N",substr($pkt,2,4));
        $out->{release}=unpack("H*",substr($pkt,24,2));
        $out->{beacon}= unpack("H*",substr($pkt,26,4));

        $out->{string}.=sprintf " t=%s (%+4d) rel=%s beacon=%s",
            strftime("%Y-%m-%d %H:%M:%S",gmtime $out->{time}),
            $out->{time}-(Time::HiRes::time+$timediff),
            $out->{release},
            resolvebeacon($out->{beacon});
    }elsif($type eq "i"){
        $out->{score}=unpack("N",substr($pkt,2,4));
        $out->{nick}= unpack("Z*",substr($pkt,6,length($pkt)-8));

        $out->{string}.=sprintf " score=%d nick=%s",
            $out->{score},
            $out->{nick};
    }elsif($type eq "B"){
        $out->{time}=unpack("N",substr($pkt,2,4));
        $out->{id}=  unpack("c",substr($pkt,6,1));
        $out->{hop}= unpack("n",substr($pkt,11,4));

        $out->{string}.=sprintf " t=%d id=%s hop=%3d",
            $out->{time},
            $out->{id},
            $out->{hop};
    }else{
        $out->{string}.= " <??: ".unpack("H*",substr($pkt,2,length($pkt)-4)).">";
    };

    my $pkt_crc=  unpack("n",substr($pkt,length($pkt)-2,2));
    my $calc_crc= crcccitt(substr($pkt,0,length($pkt)-2));
    
    if ($pkt_crc eq $calc_crc){
        $out->{crc}="ok";
    }else{
        $out->{crc}="fail";
        $out->{string}.= " CRCFAIL";
    };

    return $out;
};

sub nice_game{
    my $pkt=shift;
    my $out;
    my $type=substr($pkt,2,1);

    $out->{proto}=substr($pkt,1,1);
    $out->{type} =substr($pkt,2,1);
    $out->{id}   =unpack("V",substr($pkt,3,4));
    $out->{ctr}  =unpack("V",substr($pkt,7,4));

    $out->{string}=sprintf "G[%s] id=%d ctr=%d",
            $out->{type}, $out->{id}, $out->{ctr};

    if($type eq "A"){
        $out->{mac}      = substr($pkt,11,5);
        $out->{channel}  = unpack("C" ,substr($pkt,16,1));
        $out->{id}       = unpack("v", substr($pkt,17,2));
        $out->{flags}    = unpack("C", substr($pkt,19,1));
        $out->{flagsstr}=flagsstr($out->{flags},qw(mass short lrecv));
        $out->{interval} = unpack("C", substr($pkt,20,1));
        $out->{jitter}   = unpack("C", substr($pkt,21,1));
        $out->{title}    = unpack("Z*",substr($pkt,22,10));

        $out->{string}.=sprintf " mac=%s ch=%s id=%d fl=<%s> itvl=%d j=%d %s",
            sprint($out->{mac}),
            $out->{channel},
            $out->{id},
            $out->{flagsstr},
            $out->{interval},
            $out->{jitter},
            $out->{title};
    }else{
        $out->{string}.= " <??: ".unpack("H*",substr($pkt,2,length($pkt)-4)).">";
    };

    my $pkt_crc=  unpack("n",substr($pkt,length($pkt)-2,2));
    my $calc_crc= crcccitt(substr($pkt,0,length($pkt)-2));
    
    if ($pkt_crc eq $calc_crc){
        $out->{crc}="ok";
    }else{
        $out->{crc}="fail";
        $out->{string}.= " CRCFAIL";
    };

    return $out;
};

sub nice_beacon{
    my $pkt=shift;
    my $out;
    my $type=substr($pkt,1,1);
    $out->{type}=$type;

    if($type eq "\x17"){
        $out->{type}=    "beacon";
        $out->{length}=  unpack("C", substr($pkt,0,1));
        $out->{button}=  unpack("H*",substr($pkt,2,1));
        $out->{strength}=unpack("H*",substr($pkt,3,1));
        $out->{idx}=     unpack("N", substr($pkt,4,4));
        $out->{beacon}=  unpack("H*",substr($pkt,8,4));
        $out->{unused}=  unpack("H*",substr($pkt,12,2));

        $out->{string}=sprintf "BEACON ln=%d bt=%s str=%s idx=%08x beacon=%s",
            $out->{length},
            $out->{button},
            $out->{strength},
            $out->{idx},
            $out->{beacon};
        if(unpack("H*",substr($pkt,12,2)) ne "ffff"){
            print "unused=",unpack("H*",substr($pkt,12,2))," " if (!$quiet);
        };
    }elsif($type eq "\x23"){
        $out->{type}=   "nick";
        $out->{beacon}= unpack("H*",substr($pkt,2,4));
        $out->{nick}=   unpack("Z*",substr($pkt,6,length($pkt)-2));

        $out->{string}=sprintf "NICK beacon=%s nick=%s",
            $out->{beacon},
            $out->{nick};
    }else{
        $out->{string}="<?:".unpack("H*",$pkt).">";
    };

    my $pkt_crc=  unpack("n",substr($pkt,length($pkt)-2,2));
    my $calc_crc= crcccitt(substr($pkt,0,length($pkt)-2));
    
    if ($pkt_crc eq $calc_crc){
        $out->{crc}="ok";
    }else{
        $out->{crc}="fail";
        $out->{string}.= " CRCFAIL";
    };

    return $out;
};

sub r0ket_init{
    my $ser=shift;
    if(!defined $ser){
        if (defined $ENV{R0KETBRIDGE} && -e $ENV{R0KETBRIDGE}){
            $ser=$ENV{R0KETBRIDGE}
        };
    };
    if(!defined $ser){
        do {$ser=$_ if ( -e $_ ) } for qw(/dev/ttyS3 /dev/ttyACM0);
    };
    open($bridge, "+<",$ser) || die "open serial: $!";
    if($verbose){
        print "using: $ser\n";
    };
    return $ser;
};

sub send_raw {
    if($verbose){
        print "send: ",unpack("H*",$_[0]),"\n";
    };
    syswrite($bridge,shift);
};

sub send_pkt_num {
    my $pkt=shift;
    $pkt=~s/\\/\\\\/;
    send_raw('\\'.shift().$pkt.'\0');
};

sub send_pkt {
    send_pkt_num(shift,1);
};

sub set_txmac {
    send_pkt_num(shift,3);
};
sub set_rxmac {
    send_pkt_num(shift,4);
};
sub set_channel {
    send_pkt_num(pack("C",shift),5);
};
sub set_rxlen {
    send_pkt_num(pack("C",shift),6);
};

sub wait_ok {
    my $pkt;
    $pkt=get_packet();
    while($pkt ne "ack"){
        print "pkt=",(sprint $pkt),"\n";
        $pkt=get_packet();
    };
    print "ok!\n";
    return 1;
};
1;
