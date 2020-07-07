// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "drake_lcmt_contact_results_for_viz.h"

static int __drake_lcmt_contact_results_for_viz_hash_computed;
static uint64_t __drake_lcmt_contact_results_for_viz_hash;

uint64_t __drake_lcmt_contact_results_for_viz_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __drake_lcmt_contact_results_for_viz_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__drake_lcmt_contact_results_for_viz_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x7910f7b8bb1cdb4bLL
         + __int64_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __drake_lcmt_point_pair_contact_info_for_viz_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __drake_lcmt_hydroelastic_contact_surface_for_viz_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __drake_lcmt_contact_results_for_viz_get_hash(void)
{
    if (!__drake_lcmt_contact_results_for_viz_hash_computed) {
        __drake_lcmt_contact_results_for_viz_hash = (int64_t)__drake_lcmt_contact_results_for_viz_hash_recursive(NULL);
        __drake_lcmt_contact_results_for_viz_hash_computed = 1;
    }

    return __drake_lcmt_contact_results_for_viz_hash;
}

int __drake_lcmt_contact_results_for_viz_encode_array(void *buf, int offset, int maxlen, const drake_lcmt_contact_results_for_viz *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_point_pair_contacts), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __drake_lcmt_point_pair_contact_info_for_viz_encode_array(buf, offset + pos, maxlen - pos, p[element].point_pair_contact_info, p[element].num_point_pair_contacts);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_hydroelastic_contacts), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __drake_lcmt_hydroelastic_contact_surface_for_viz_encode_array(buf, offset + pos, maxlen - pos, p[element].hydroelastic_contacts, p[element].num_hydroelastic_contacts);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int drake_lcmt_contact_results_for_viz_encode(void *buf, int offset, int maxlen, const drake_lcmt_contact_results_for_viz *p)
{
    int pos = 0, thislen;
    int64_t hash = __drake_lcmt_contact_results_for_viz_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __drake_lcmt_contact_results_for_viz_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __drake_lcmt_contact_results_for_viz_encoded_array_size(const drake_lcmt_contact_results_for_viz *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].timestamp), 1);

        size += __int32_t_encoded_array_size(&(p[element].num_point_pair_contacts), 1);

        size += __drake_lcmt_point_pair_contact_info_for_viz_encoded_array_size(p[element].point_pair_contact_info, p[element].num_point_pair_contacts);

        size += __int32_t_encoded_array_size(&(p[element].num_hydroelastic_contacts), 1);

        size += __drake_lcmt_hydroelastic_contact_surface_for_viz_encoded_array_size(p[element].hydroelastic_contacts, p[element].num_hydroelastic_contacts);

    }
    return size;
}

int drake_lcmt_contact_results_for_viz_encoded_size(const drake_lcmt_contact_results_for_viz *p)
{
    return 8 + __drake_lcmt_contact_results_for_viz_encoded_array_size(p, 1);
}

int __drake_lcmt_contact_results_for_viz_decode_array(const void *buf, int offset, int maxlen, drake_lcmt_contact_results_for_viz *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_point_pair_contacts), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].point_pair_contact_info = (drake_lcmt_point_pair_contact_info_for_viz*) lcm_malloc(sizeof(drake_lcmt_point_pair_contact_info_for_viz) * p[element].num_point_pair_contacts);
        thislen = __drake_lcmt_point_pair_contact_info_for_viz_decode_array(buf, offset + pos, maxlen - pos, p[element].point_pair_contact_info, p[element].num_point_pair_contacts);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_hydroelastic_contacts), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].hydroelastic_contacts = (drake_lcmt_hydroelastic_contact_surface_for_viz*) lcm_malloc(sizeof(drake_lcmt_hydroelastic_contact_surface_for_viz) * p[element].num_hydroelastic_contacts);
        thislen = __drake_lcmt_hydroelastic_contact_surface_for_viz_decode_array(buf, offset + pos, maxlen - pos, p[element].hydroelastic_contacts, p[element].num_hydroelastic_contacts);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __drake_lcmt_contact_results_for_viz_decode_array_cleanup(drake_lcmt_contact_results_for_viz *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].timestamp), 1);

        __int32_t_decode_array_cleanup(&(p[element].num_point_pair_contacts), 1);

        __drake_lcmt_point_pair_contact_info_for_viz_decode_array_cleanup(p[element].point_pair_contact_info, p[element].num_point_pair_contacts);
        if (p[element].point_pair_contact_info) free(p[element].point_pair_contact_info);

        __int32_t_decode_array_cleanup(&(p[element].num_hydroelastic_contacts), 1);

        __drake_lcmt_hydroelastic_contact_surface_for_viz_decode_array_cleanup(p[element].hydroelastic_contacts, p[element].num_hydroelastic_contacts);
        if (p[element].hydroelastic_contacts) free(p[element].hydroelastic_contacts);

    }
    return 0;
}

int drake_lcmt_contact_results_for_viz_decode(const void *buf, int offset, int maxlen, drake_lcmt_contact_results_for_viz *p)
{
    int pos = 0, thislen;
    int64_t hash = __drake_lcmt_contact_results_for_viz_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __drake_lcmt_contact_results_for_viz_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int drake_lcmt_contact_results_for_viz_decode_cleanup(drake_lcmt_contact_results_for_viz *p)
{
    return __drake_lcmt_contact_results_for_viz_decode_array_cleanup(p, 1);
}

int __drake_lcmt_contact_results_for_viz_clone_array(const drake_lcmt_contact_results_for_viz *p, drake_lcmt_contact_results_for_viz *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].timestamp), &(q[element].timestamp), 1);

        __int32_t_clone_array(&(p[element].num_point_pair_contacts), &(q[element].num_point_pair_contacts), 1);

        q[element].point_pair_contact_info = (drake_lcmt_point_pair_contact_info_for_viz*) lcm_malloc(sizeof(drake_lcmt_point_pair_contact_info_for_viz) * q[element].num_point_pair_contacts);
        __drake_lcmt_point_pair_contact_info_for_viz_clone_array(p[element].point_pair_contact_info, q[element].point_pair_contact_info, p[element].num_point_pair_contacts);

        __int32_t_clone_array(&(p[element].num_hydroelastic_contacts), &(q[element].num_hydroelastic_contacts), 1);

        q[element].hydroelastic_contacts = (drake_lcmt_hydroelastic_contact_surface_for_viz*) lcm_malloc(sizeof(drake_lcmt_hydroelastic_contact_surface_for_viz) * q[element].num_hydroelastic_contacts);
        __drake_lcmt_hydroelastic_contact_surface_for_viz_clone_array(p[element].hydroelastic_contacts, q[element].hydroelastic_contacts, p[element].num_hydroelastic_contacts);

    }
    return 0;
}

drake_lcmt_contact_results_for_viz *drake_lcmt_contact_results_for_viz_copy(const drake_lcmt_contact_results_for_viz *p)
{
    drake_lcmt_contact_results_for_viz *q = (drake_lcmt_contact_results_for_viz*) malloc(sizeof(drake_lcmt_contact_results_for_viz));
    __drake_lcmt_contact_results_for_viz_clone_array(p, q, 1);
    return q;
}

void drake_lcmt_contact_results_for_viz_destroy(drake_lcmt_contact_results_for_viz *p)
{
    __drake_lcmt_contact_results_for_viz_decode_array_cleanup(p, 1);
    free(p);
}

int drake_lcmt_contact_results_for_viz_publish(lcm_t *lc, const char *channel, const drake_lcmt_contact_results_for_viz *p)
{
      int max_data_size = drake_lcmt_contact_results_for_viz_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = drake_lcmt_contact_results_for_viz_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _drake_lcmt_contact_results_for_viz_subscription_t {
    drake_lcmt_contact_results_for_viz_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void drake_lcmt_contact_results_for_viz_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    drake_lcmt_contact_results_for_viz p;
    memset(&p, 0, sizeof(drake_lcmt_contact_results_for_viz));
    status = drake_lcmt_contact_results_for_viz_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding drake_lcmt_contact_results_for_viz!!!\n", status);
        return;
    }

    drake_lcmt_contact_results_for_viz_subscription_t *h = (drake_lcmt_contact_results_for_viz_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    drake_lcmt_contact_results_for_viz_decode_cleanup (&p);
}

drake_lcmt_contact_results_for_viz_subscription_t* drake_lcmt_contact_results_for_viz_subscribe (lcm_t *lcm,
                    const char *channel,
                    drake_lcmt_contact_results_for_viz_handler_t f, void *userdata)
{
    drake_lcmt_contact_results_for_viz_subscription_t *n = (drake_lcmt_contact_results_for_viz_subscription_t*)
                       malloc(sizeof(drake_lcmt_contact_results_for_viz_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 drake_lcmt_contact_results_for_viz_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg drake_lcmt_contact_results_for_viz LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int drake_lcmt_contact_results_for_viz_subscription_set_queue_capacity (drake_lcmt_contact_results_for_viz_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int drake_lcmt_contact_results_for_viz_unsubscribe(lcm_t *lcm, drake_lcmt_contact_results_for_viz_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe drake_lcmt_contact_results_for_viz_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}
