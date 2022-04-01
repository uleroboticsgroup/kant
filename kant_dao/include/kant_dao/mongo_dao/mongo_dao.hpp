
#ifndef KANT_MONGO_DAO_HPP
#define KANT_MONGO_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>

#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/json.hpp>

#include "kant_dto/dto.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoDao {

private:
  mongocxx::client client;
  std::string mongo_uri;
  std::string database_name;
  std::string collection_name;

protected:
  mongocxx::collection get_collection() {
    return this->client[this->database_name][this->collection_name];
  }

  virtual bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto) = 0;

  bool mongo_insert(kant::dto::Dto *dto) {

    auto doc_value = this->dto_to_mongo(dto);

    bsoncxx::stdx::optional<mongocxx::result::insert_one> result =
        this->get_collection().insert_one(doc_value.view());

    return result.value().result().inserted_count() == 1;
  };

  bool mongo_delete(kant::dto::Dto *dto) {
    auto doc_value = this->dto_to_mongo(dto);

    bsoncxx::stdx::optional<mongocxx::result::delete_result> result =
        this->get_collection().delete_one(doc_value.view());

    return result.value().result().deleted_count() == 1;
  };

public:
  MongoDao(std::string mongo_uri, std::string collection_name,
           bool init_instance) {

    if (init_instance) {
      mongocxx::instance instance{}; // This should be done only once.
    }

    this->mongo_uri = mongo_uri;
    mongocxx::uri uri(mongo_uri);
    this->client = mongocxx::client(uri);

    this->database_name = uri.database();
    this->collection_name = collection_name;
  };

  virtual ~MongoDao(){};

  std::string get_uri() { return this->mongo_uri; }
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
