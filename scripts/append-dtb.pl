#!/usr/bin/perl

use warnings;
use strict;

my @bytes;
foreach (@ARGV) {
	open my $file, '<', $_ or die "$_: $!";
	push @bytes, map { split //, $_ } <$file>;
}

my $len = $#bytes + 1;
# 0x2c = zImage length
$bytes[0x2c] = chr(($len >> 0) & 0xff);
$bytes[0x2d] = chr(($len >> 8) & 0xff);
$bytes[0x2e] = chr(($len >> 16) & 0xff);
$bytes[0x2f] = chr(($len >> 24) & 0xff);

print @bytes;
