#!/usr/bin/perl -w
use XML::DOM;
use XML::DOM::XPath;

my $nmax = 32;

foreach my $file (<*-1.scn>) {

print $file."\n";

my $parser = XML::DOM::Parser->new();
my $doc = $parser->parsefile($file);

for (my $i = 2; $i <= $nmax; $i++) {

# find the M1 Node
my @nodelist = $doc->findnodes( q{//Node[@name='M1']});

foreach my $m1 (@nodelist) {
    my $clone = $m1->cloneNode(1);
    $clone->setAttribute('name',"M$i");
    # translate the object
    foreach my $grid ($clone->findnodes(q{descendant::Object[@type='RegularGrid']})) {
	$grid->setAttribute('xmin',$grid->getAttribute('xmin') + ($i-1)*5);
	$grid->setAttribute('xmax',$grid->getAttribute('xmax') + ($i-1)*5);
    }
    #$_->setValue($_->getValue + ($i-1)*5) foreach ($clone->findnodes('@xmin'));
    #$_->setValue($_->getValue + ($i-1)*5) foreach ($clone->findnodes('@xmax'));
    $m1->getParentNode->appendChild($clone);
}
# only write a file at every power-of-two
if ((($i-1)&$i)==0) {
my $newfile = $file;
$newfile=~s/-1.scn$/-$i.scn/;
print " -> $newfile.\n";
$doc->printToFile($newfile);
}
}
}

