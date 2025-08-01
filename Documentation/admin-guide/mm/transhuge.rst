============================
Transparent Hugepage Support
============================

Objective
=========

Performance critical computing applications dealing with large memory
working sets are already running on top of libhugetlbfs and in turn
hugetlbfs. Transparent HugePage Support (THP) is an alternative mean of
using huge pages for the backing of virtual memory with huge pages
that supports the automatic promotion and demotion of page sizes and
without the shortcomings of hugetlbfs.

Currently THP only works for anonymous memory mappings and tmpfs/shmem.
But in the future it can expand to other filesystems.

.. note::
   in the examples below we presume that the basic page size is 4K and
   the huge page size is 2M, although the actual numbers may vary
   depending on the CPU architecture.

The reason applications are running faster is because of two
factors. The first factor is almost completely irrelevant and it's not
of significant interest because it'll also have the downside of
requiring larger clear-page copy-page in page faults which is a
potentially negative effect. The first factor consists in taking a
single page fault for each 2M virtual region touched by userland (so
reducing the enter/exit kernel frequency by a 512 times factor). This
only matters the first time the memory is accessed for the lifetime of
a memory mapping. The second long lasting and much more important
factor will affect all subsequent accesses to the memory for the whole
runtime of the application. The second factor consist of two
components:

1) the TLB miss will run faster (especially with virtualization using
   nested pagetables but almost always also on bare metal without
   virtualization)

2) a single TLB entry will be mapping a much larger amount of virtual
   memory in turn reducing the number of TLB misses. With
   virtualization and nested pagetables the TLB can be mapped of
   larger size only if both KVM and the Linux guest are using
   hugepages but a significant speedup already happens if only one of
   the two is using hugepages just because of the fact the TLB miss is
   going to run faster.

Modern kernels support "multi-size THP" (mTHP), which introduces the
ability to allocate memory in blocks that are bigger than a base page
but smaller than traditional PMD-size (as described above), in
increments of a power-of-2 number of pages. mTHP can back anonymous
memory (for example 16K, 32K, 64K, etc). These THPs continue to be
PTE-mapped, but in many cases can still provide similar benefits to
those outlined above: Page faults are significantly reduced (by a
factor of e.g. 4, 8, 16, etc), but latency spikes are much less
prominent because the size of each page isn't as huge as the PMD-sized
variant and there is less memory to clear in each page fault. Some
architectures also employ TLB compression mechanisms to squeeze more
entries in when a set of PTEs are virtually and physically contiguous
and approporiately aligned. In this case, TLB misses will occur less
often.

THP can be enabled system wide or restricted to certain tasks or even
memory ranges inside task's address space. Unless THP is completely
disabled, there is ``khugepaged`` daemon that scans memory and
collapses sequences of basic pages into PMD-sized huge pages.

The THP behaviour is controlled via :ref:`sysfs <thp_sysfs>`
interface and using madvise(2) and prctl(2) system calls.

Transparent Hugepage Support maximizes the usefulness of free memory
if compared to the reservation approach of hugetlbfs by allowing all
unused memory to be used as cache or other movable (or even unmovable
entities). It doesn't require reservation to prevent hugepage
allocation failures to be noticeable from userland. It allows paging
and all other advanced VM features to be available on the
hugepages. It requires no modifications for applications to take
advantage of it.

Applications however can be further optimized to take advantage of
this feature, like for example they've been optimized before to avoid
a flood of mmap system calls for every malloc(4k). Optimizing userland
is by far not mandatory and khugepaged already can take care of long
lived page allocations even for hugepage unaware applications that
deals with large amounts of memory.

In certain cases when hugepages are enabled system wide, application
may end up allocating more memory resources. An application may mmap a
large region but only touch 1 byte of it, in that case a 2M page might
be allocated instead of a 4k page for no good. This is why it's
possible to disable hugepages system-wide and to only have them inside
MADV_HUGEPAGE madvise regions.

Embedded systems should enable hugepages only inside madvise regions
to eliminate any risk of wasting any precious byte of memory and to
only run faster.

Applications that gets a lot of benefit from hugepages and that don't
risk to lose memory by using hugepages, should use
madvise(MADV_HUGEPAGE) on their critical mmapped regions.

.. _thp_sysfs:

sysfs
=====

Global THP controls
-------------------

Transparent Hugepage Support for anonymous memory can be disabled
(mostly for debugging purposes) or only enabled inside MADV_HUGEPAGE
regions (to avoid the risk of consuming more memory resources) or enabled
system wide. This can be achieved per-supported-THP-size with one of::

	echo always >/sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/enabled
	echo madvise >/sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/enabled
	echo never >/sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/enabled

where <size> is the hugepage size being addressed, the available sizes
for which vary by system.

.. note:: Setting "never" in all sysfs THP controls does **not** disable
          Transparent Huge Pages globally. This is because ``madvise(...,
          MADV_COLLAPSE)`` ignores these settings and collapses ranges to
          PMD-sized huge pages unconditionally.

For example::

	echo always >/sys/kernel/mm/transparent_hugepage/hugepages-2048kB/enabled

Alternatively it is possible to specify that a given hugepage size
will inherit the top-level "enabled" value::

	echo inherit >/sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/enabled

For example::

	echo inherit >/sys/kernel/mm/transparent_hugepage/hugepages-2048kB/enabled

The top-level setting (for use with "inherit") can be set by issuing
one of the following commands::

	echo always >/sys/kernel/mm/transparent_hugepage/enabled
	echo madvise >/sys/kernel/mm/transparent_hugepage/enabled
	echo never >/sys/kernel/mm/transparent_hugepage/enabled

By default, PMD-sized hugepages have enabled="inherit" and all other
hugepage sizes have enabled="never". If enabling multiple hugepage
sizes, the kernel will select the most appropriate enabled size for a
given allocation.

It's also possible to limit defrag efforts in the VM to generate
anonymous hugepages in case they're not immediately free to madvise
regions or to never try to defrag memory and simply fallback to regular
pages unless hugepages are immediately available. Clearly if we spend CPU
time to defrag memory, we would expect to gain even more by the fact we
use hugepages later instead of regular pages. This isn't always
guaranteed, but it may be more likely in case the allocation is for a
MADV_HUGEPAGE region.

::

	echo always >/sys/kernel/mm/transparent_hugepage/defrag
	echo defer >/sys/kernel/mm/transparent_hugepage/defrag
	echo defer+madvise >/sys/kernel/mm/transparent_hugepage/defrag
	echo madvise >/sys/kernel/mm/transparent_hugepage/defrag
	echo never >/sys/kernel/mm/transparent_hugepage/defrag

always
	means that an application requesting THP will stall on
	allocation failure and directly reclaim pages and compact
	memory in an effort to allocate a THP immediately. This may be
	desirable for virtual machines that benefit heavily from THP
	use and are willing to delay the VM start to utilise them.

defer
	means that an application will wake kswapd in the background
	to reclaim pages and wake kcompactd to compact memory so that
	THP is available in the near future. It's the responsibility
	of khugepaged to then install the THP pages later.

defer+madvise
	will enter direct reclaim and compaction like ``always``, but
	only for regions that have used madvise(MADV_HUGEPAGE); all
	other regions will wake kswapd in the background to reclaim
	pages and wake kcompactd to compact memory so that THP is
	available in the near future.

madvise
	will enter direct reclaim like ``always`` but only for regions
	that are have used madvise(MADV_HUGEPAGE). This is the default
	behaviour.

never
	should be self-explanatory. Note that ``madvise(...,
	MADV_COLLAPSE)`` can still cause transparent huge pages to be
	obtained even if this mode is specified everywhere.

By default kernel tries to use huge, PMD-mappable zero page on read
page fault to anonymous mapping. It's possible to disable huge zero
page by writing 0 or enable it back by writing 1::

	echo 0 >/sys/kernel/mm/transparent_hugepage/use_zero_page
	echo 1 >/sys/kernel/mm/transparent_hugepage/use_zero_page

Some userspace (such as a test program, or an optimized memory
allocation library) may want to know the size (in bytes) of a
PMD-mappable transparent hugepage::

	cat /sys/kernel/mm/transparent_hugepage/hpage_pmd_size

All THPs at fault and collapse time will be added to _deferred_list,
and will therefore be split under memory presure if they are considered
"underused". A THP is underused if the number of zero-filled pages in
the THP is above max_ptes_none (see below). It is possible to disable
this behaviour by writing 0 to shrink_underused, and enable it by writing
1 to it::

	echo 0 > /sys/kernel/mm/transparent_hugepage/shrink_underused
	echo 1 > /sys/kernel/mm/transparent_hugepage/shrink_underused

khugepaged will be automatically started when PMD-sized THP is enabled
(either of the per-size anon control or the top-level control are set
to "always" or "madvise"), and it'll be automatically shutdown when
PMD-sized THP is disabled (when both the per-size anon control and the
top-level control are "never")

Khugepaged controls
-------------------

.. note::
   khugepaged currently only searches for opportunities to collapse to
   PMD-sized THP and no attempt is made to collapse to other THP
   sizes.

khugepaged runs usually at low frequency so while one may not want to
invoke defrag algorithms synchronously during the page faults, it
should be worth invoking defrag at least in khugepaged. However it's
also possible to disable defrag in khugepaged by writing 0 or enable
defrag in khugepaged by writing 1::

	echo 0 >/sys/kernel/mm/transparent_hugepage/khugepaged/defrag
	echo 1 >/sys/kernel/mm/transparent_hugepage/khugepaged/defrag

You can also control how many pages khugepaged should scan at each
pass::

	/sys/kernel/mm/transparent_hugepage/khugepaged/pages_to_scan

and how many milliseconds to wait in khugepaged between each pass (you
can set this to 0 to run khugepaged at 100% utilization of one core)::

	/sys/kernel/mm/transparent_hugepage/khugepaged/scan_sleep_millisecs

and how many milliseconds to wait in khugepaged if there's an hugepage
allocation failure to throttle the next allocation attempt::

	/sys/kernel/mm/transparent_hugepage/khugepaged/alloc_sleep_millisecs

The khugepaged progress can be seen in the number of pages collapsed (note
that this counter may not be an exact count of the number of pages
collapsed, since "collapsed" could mean multiple things: (1) A PTE mapping
being replaced by a PMD mapping, or (2) All 4K physical pages replaced by
one 2M hugepage. Each may happen independently, or together, depending on
the type of memory and the failures that occur. As such, this value should
be interpreted roughly as a sign of progress, and counters in /proc/vmstat
consulted for more accurate accounting)::

	/sys/kernel/mm/transparent_hugepage/khugepaged/pages_collapsed

for each pass::

	/sys/kernel/mm/transparent_hugepage/khugepaged/full_scans

``max_ptes_none`` specifies how many extra small pages (that are
not already mapped) can be allocated when collapsing a group
of small pages into one large page::

	/sys/kernel/mm/transparent_hugepage/khugepaged/max_ptes_none

A higher value leads to use additional memory for programs.
A lower value leads to gain less thp performance. Value of
max_ptes_none can waste cpu time very little, you can
ignore it.

``max_ptes_swap`` specifies how many pages can be brought in from
swap when collapsing a group of pages into a transparent huge page::

	/sys/kernel/mm/transparent_hugepage/khugepaged/max_ptes_swap

A higher value can cause excessive swap IO and waste
memory. A lower value can prevent THPs from being
collapsed, resulting fewer pages being collapsed into
THPs, and lower memory access performance.

``max_ptes_shared`` specifies how many pages can be shared across multiple
processes. khugepaged might treat pages of THPs as shared if any page of
that THP is shared. Exceeding the number would block the collapse::

	/sys/kernel/mm/transparent_hugepage/khugepaged/max_ptes_shared

A higher value may increase memory footprint for some workloads.

Boot parameters
===============

You can change the sysfs boot time default for the top-level "enabled"
control by passing the parameter ``transparent_hugepage=always`` or
``transparent_hugepage=madvise`` or ``transparent_hugepage=never`` to the
kernel command line.

Alternatively, each supported anonymous THP size can be controlled by
passing ``thp_anon=<size>[KMG],<size>[KMG]:<state>;<size>[KMG]-<size>[KMG]:<state>``,
where ``<size>`` is the THP size (must be a power of 2 of PAGE_SIZE and
supported anonymous THP)  and ``<state>`` is one of ``always``, ``madvise``,
``never`` or ``inherit``.

For example, the following will set 16K, 32K, 64K THP to ``always``,
set 128K, 512K to ``inherit``, set 256K to ``madvise`` and 1M, 2M
to ``never``::

	thp_anon=16K-64K:always;128K,512K:inherit;256K:madvise;1M-2M:never

``thp_anon=`` may be specified multiple times to configure all THP sizes as
required. If ``thp_anon=`` is specified at least once, any anon THP sizes
not explicitly configured on the command line are implicitly set to
``never``.

``transparent_hugepage`` setting only affects the global toggle. If
``thp_anon`` is not specified, PMD_ORDER THP will default to ``inherit``.
However, if a valid ``thp_anon`` setting is provided by the user, the
PMD_ORDER THP policy will be overridden. If the policy for PMD_ORDER
is not defined within a valid ``thp_anon``, its policy will default to
``never``.

Similarly to ``transparent_hugepage``, you can control the hugepage
allocation policy for the internal shmem mount by using the kernel parameter
``transparent_hugepage_shmem=<policy>``, where ``<policy>`` is one of the
seven valid policies for shmem (``always``, ``within_size``, ``advise``,
``never``, ``deny``, and ``force``).

Similarly to ``transparent_hugepage_shmem``, you can control the default
hugepage allocation policy for the tmpfs mount by using the kernel parameter
``transparent_hugepage_tmpfs=<policy>``, where ``<policy>`` is one of the
four valid policies for tmpfs (``always``, ``within_size``, ``advise``,
``never``). The tmpfs mount default policy is ``never``.

In the same manner as ``thp_anon`` controls each supported anonymous THP
size, ``thp_shmem`` controls each supported shmem THP size. ``thp_shmem``
has the same format as ``thp_anon``, but also supports the policy
``within_size``.

``thp_shmem=`` may be specified multiple times to configure all THP sizes
as required. If ``thp_shmem=`` is specified at least once, any shmem THP
sizes not explicitly configured on the command line are implicitly set to
``never``.

``transparent_hugepage_shmem`` setting only affects the global toggle. If
``thp_shmem`` is not specified, PMD_ORDER hugepage will default to
``inherit``. However, if a valid ``thp_shmem`` setting is provided by the
user, the PMD_ORDER hugepage policy will be overridden. If the policy for
PMD_ORDER is not defined within a valid ``thp_shmem``, its policy will
default to ``never``.

Hugepages in tmpfs/shmem
========================

Traditionally, tmpfs only supported a single huge page size ("PMD"). Today,
it also supports smaller sizes just like anonymous memory, often referred
to as "multi-size THP" (mTHP). Huge pages of any size are commonly
represented in the kernel as "large folios".

While there is fine control over the huge page sizes to use for the internal
shmem mount (see below), ordinary tmpfs mounts will make use of all available
huge page sizes without any control over the exact sizes, behaving more like
other file systems.

tmpfs mounts
------------

The THP allocation policy for tmpfs mounts can be adjusted using the mount
option: ``huge=``. It can have following values:

always
    Attempt to allocate huge pages every time we need a new page;

never
    Do not allocate huge pages. Note that ``madvise(..., MADV_COLLAPSE)``
    can still cause transparent huge pages to be obtained even if this mode
    is specified everywhere;

within_size
    Only allocate huge page if it will be fully within i_size.
    Also respect madvise() hints;

advise
    Only allocate huge pages if requested with madvise();

Remember, that the kernel may use huge pages of all available sizes, and
that no fine control as for the internal tmpfs mount is available.

The default policy in the past was ``never``, but it can now be adjusted
using the kernel parameter ``transparent_hugepage_tmpfs=<policy>``.

``mount -o remount,huge= /mountpoint`` works fine after mount: remounting
``huge=never`` will not attempt to break up huge pages at all, just stop more
from being allocated.

In addition to policies listed above, the sysfs knob
/sys/kernel/mm/transparent_hugepage/shmem_enabled will affect the
allocation policy of tmpfs mounts, when set to the following values:

deny
    For use in emergencies, to force the huge option off from
    all mounts;
force
    Force the huge option on for all - very useful for testing;

shmem / internal tmpfs
----------------------
The mount internal tmpfs mount is used for SysV SHM, memfds, shared anonymous
mmaps (of /dev/zero or MAP_ANONYMOUS), GPU drivers' DRM  objects, Ashmem.

To control the THP allocation policy for this internal tmpfs mount, the
sysfs knob /sys/kernel/mm/transparent_hugepage/shmem_enabled and the knobs
per THP size in
'/sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/shmem_enabled'
can be used.

The global knob has the same semantics as the ``huge=`` mount options
for tmpfs mounts, except that the different huge page sizes can be controlled
individually, and will only use the setting of the global knob when the
per-size knob is set to 'inherit'.

The options 'force' and 'deny' are dropped for the individual sizes, which
are rather testing artifacts from the old ages.

always
    Attempt to allocate <size> huge pages every time we need a new page;

inherit
    Inherit the top-level "shmem_enabled" value. By default, PMD-sized hugepages
    have enabled="inherit" and all other hugepage sizes have enabled="never";

never
    Do not allocate <size> huge pages. Note that ``madvise(...,
    MADV_COLLAPSE)`` can still cause transparent huge pages to be obtained
    even if this mode is specified everywhere;

within_size
    Only allocate <size> huge page if it will be fully within i_size.
    Also respect madvise() hints;

advise
    Only allocate <size> huge pages if requested with madvise();

Need of application restart
===========================

The transparent_hugepage/enabled and
transparent_hugepage/hugepages-<size>kB/enabled values and tmpfs mount
option only affect future behavior. So to make them effective you need
to restart any application that could have been using hugepages. This
also applies to the regions registered in khugepaged.

Monitoring usage
================

The number of PMD-sized anonymous transparent huge pages currently used by the
system is available by reading the AnonHugePages field in ``/proc/meminfo``.
To identify what applications are using PMD-sized anonymous transparent huge
pages, it is necessary to read ``/proc/PID/smaps`` and count the AnonHugePages
fields for each mapping. (Note that AnonHugePages only applies to traditional
PMD-sized THP for historical reasons and should have been called
AnonHugePmdMapped).

The number of file transparent huge pages mapped to userspace is available
by reading ShmemPmdMapped and ShmemHugePages fields in ``/proc/meminfo``.
To identify what applications are mapping file transparent huge pages, it
is necessary to read ``/proc/PID/smaps`` and count the FilePmdMapped fields
for each mapping.

Note that reading the smaps file is expensive and reading it
frequently will incur overhead.

There are a number of counters in ``/proc/vmstat`` that may be used to
monitor how successfully the system is providing huge pages for use.

thp_fault_alloc
	is incremented every time a huge page is successfully
	allocated and charged to handle a page fault.

thp_collapse_alloc
	is incremented by khugepaged when it has found
	a range of pages to collapse into one huge page and has
	successfully allocated a new huge page to store the data.

thp_fault_fallback
	is incremented if a page fault fails to allocate or charge
	a huge page and instead falls back to using small pages.

thp_fault_fallback_charge
	is incremented if a page fault fails to charge a huge page and
	instead falls back to using small pages even though the
	allocation was successful.

thp_collapse_alloc_failed
	is incremented if khugepaged found a range
	of pages that should be collapsed into one huge page but failed
	the allocation.

thp_file_alloc
	is incremented every time a shmem huge page is successfully
	allocated (Note that despite being named after "file", the counter
	measures only shmem).

thp_file_fallback
	is incremented if a shmem huge page is attempted to be allocated
	but fails and instead falls back to using small pages. (Note that
	despite being named after "file", the counter measures only shmem).

thp_file_fallback_charge
	is incremented if a shmem huge page cannot be charged and instead
	falls back to using small pages even though the allocation was
	successful. (Note that despite being named after "file", the
	counter measures only shmem).

thp_file_mapped
	is incremented every time a file or shmem huge page is mapped into
	user address space.

thp_split_page
	is incremented every time a huge page is split into base
	pages. This can happen for a variety of reasons but a common
	reason is that a huge page is old and is being reclaimed.
	This action implies splitting all PMD the page mapped with.

thp_split_page_failed
	is incremented if kernel fails to split huge
	page. This can happen if the page was pinned by somebody.

thp_deferred_split_page
	is incremented when a huge page is put onto split
	queue. This happens when a huge page is partially unmapped and
	splitting it would free up some memory. Pages on split queue are
	going to be split under memory pressure.

thp_underused_split_page
	is incremented when a huge page on the split queue was split
	because it was underused. A THP is underused if the number of
	zero pages in the THP is above a certain threshold
	(/sys/kernel/mm/transparent_hugepage/khugepaged/max_ptes_none).

thp_split_pmd
	is incremented every time a PMD split into table of PTEs.
	This can happen, for instance, when application calls mprotect() or
	munmap() on part of huge page. It doesn't split huge page, only
	page table entry.

thp_zero_page_alloc
	is incremented every time a huge zero page used for thp is
	successfully allocated. Note, it doesn't count every map of
	the huge zero page, only its allocation.

thp_zero_page_alloc_failed
	is incremented if kernel fails to allocate
	huge zero page and falls back to using small pages.

thp_swpout
	is incremented every time a huge page is swapout in one
	piece without splitting.

thp_swpout_fallback
	is incremented if a huge page has to be split before swapout.
	Usually because failed to allocate some continuous swap space
	for the huge page.

In /sys/kernel/mm/transparent_hugepage/hugepages-<size>kB/stats, There are
also individual counters for each huge page size, which can be utilized to
monitor the system's effectiveness in providing huge pages for usage. Each
counter has its own corresponding file.

anon_fault_alloc
	is incremented every time a huge page is successfully
	allocated and charged to handle a page fault.

anon_fault_fallback
	is incremented if a page fault fails to allocate or charge
	a huge page and instead falls back to using huge pages with
	lower orders or small pages.

anon_fault_fallback_charge
	is incremented if a page fault fails to charge a huge page and
	instead falls back to using huge pages with lower orders or
	small pages even though the allocation was successful.

zswpout
	is incremented every time a huge page is swapped out to zswap in one
	piece without splitting.

swpin
	is incremented every time a huge page is swapped in from a non-zswap
	swap device in one piece.

swpin_fallback
	is incremented if swapin fails to allocate or charge a huge page
	and instead falls back to using huge pages with lower orders or
	small pages.

swpin_fallback_charge
	is incremented if swapin fails to charge a huge page and instead
	falls back to using  huge pages with lower orders or small pages
	even though the allocation was successful.

swpout
	is incremented every time a huge page is swapped out to a non-zswap
	swap device in one piece without splitting.

swpout_fallback
	is incremented if a huge page has to be split before swapout.
	Usually because failed to allocate some continuous swap space
	for the huge page.

shmem_alloc
	is incremented every time a shmem huge page is successfully
	allocated.

shmem_fallback
	is incremented if a shmem huge page is attempted to be allocated
	but fails and instead falls back to using small pages.

shmem_fallback_charge
	is incremented if a shmem huge page cannot be charged and instead
	falls back to using small pages even though the allocation was
	successful.

split
	is incremented every time a huge page is successfully split into
	smaller orders. This can happen for a variety of reasons but a
	common reason is that a huge page is old and is being reclaimed.

split_failed
	is incremented if kernel fails to split huge
	page. This can happen if the page was pinned by somebody.

split_deferred
        is incremented when a huge page is put onto split queue.
        This happens when a huge page is partially unmapped and splitting
        it would free up some memory. Pages on split queue are going to
        be split under memory pressure, if splitting is possible.

nr_anon
       the number of anonymous THP we have in the whole system. These THPs
       might be currently entirely mapped or have partially unmapped/unused
       subpages.

nr_anon_partially_mapped
       the number of anonymous THP which are likely partially mapped, possibly
       wasting memory, and have been queued for deferred memory reclamation.
       Note that in corner some cases (e.g., failed migration), we might detect
       an anonymous THP as "partially mapped" and count it here, even though it
       is not actually partially mapped anymore.

As the system ages, allocating huge pages may be expensive as the
system uses memory compaction to copy data around memory to free a
huge page for use. There are some counters in ``/proc/vmstat`` to help
monitor this overhead.

compact_stall
	is incremented every time a process stalls to run
	memory compaction so that a huge page is free for use.

compact_success
	is incremented if the system compacted memory and
	freed a huge page for use.

compact_fail
	is incremented if the system tries to compact memory
	but failed.

It is possible to establish how long the stalls were using the function
tracer to record how long was spent in __alloc_pages() and
using the mm_page_alloc tracepoint to identify which allocations were
for huge pages.

Optimizing the applications
===========================

To be guaranteed that the kernel will map a THP immediately in any
memory region, the mmap region has to be hugepage naturally
aligned. posix_memalign() can provide that guarantee.

Hugetlbfs
=========

You can use hugetlbfs on a kernel that has transparent hugepage
support enabled just fine as always. No difference can be noted in
hugetlbfs other than there will be less overall fragmentation. All
usual features belonging to hugetlbfs are preserved and
unaffected. libhugetlbfs will also work fine as usual.
