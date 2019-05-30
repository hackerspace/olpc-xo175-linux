#!/usr/bin/perl -n

if (/^--- a\/(.*)/) {
	$oldfile = $1;
	$newfile = undef;
	next;
}

if (/^\+\+\+ b\/(.*)/) {
	$newfile = $1;
	if ($oldfile eq '/dev/null') {
		$files{$newfile} = 'NEW';
	} else {
		$files{$newfile} = 'OLD';
	}

	next;
}

if (/^\+config (.*)/) {
	$config = $1;
	$configs{$config} //= {};
	push @{$configs{$config}{Kconfig}}, $newfile;
	next;
}

if (/^\+.*\$\(CONFIG_([^\)]+)\)\s*\S*=\s*(.*)/) {
	$config = $1;
	@objects = split /\s+/, $2;
	$configs{$config} //= {};
	push @{$configs{$config}{obj}}, { $newfile => [ @objects ] };
}

if (/^\+\s*#if\s*(.*)/) {
	@configs = grep /^CONFIG_/, split /[\s*\(\)\|]+/, $1;
	foreach $config (@configs) {
		$configs{$config} //= {};
		push @{$configs{$config}{related}}, @configs;
		push @{$configs{$config}{ifdef}}, $newfile;
	}
}

use Data::Dumper;
$Data::Dumper::Deepcopy = 1;
$Data::Dumper::Sortkeys = 1;

END {
	warn Dumper \%configs;
}
